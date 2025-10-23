import json
from paho.mqtt import client as mqtt
import multiprocessing.shared_memory as sm

import os
from datetime import datetime
import numpy as np

from dotenv import load_dotenv
load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))

# USER_UUID = "d514aa76-a6cf-4718-9a8d-db69e330229e-liust"  # VR
USER_UUID = "42e98ca9-5d60-405b-8cb9-d1ed9f1c26a5-liust" # Browser
ROBOT_UUID = os.getenv("ROBOT_UUID", "no-uuid")
ROBOT_MODEL = os.getenv("ROBOT_MODEL", "piper")
MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "dev")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev") + "/" + ROBOT_UUID

class MQTT_Client():
    def __init__(self, joint_topic, tool_topic, mode):
        self.joint_topic = joint_topic
        self.tool_topic = tool_topic
        self.mode = mode

        self.client = mqtt.Client(transport="websockets")

        self.MQTT_LOCAL_SERVER = "192.168.197.42"
        self.MQTT_LOCAL_PORT = 8333
        self.MQTT_UCLAB_SERVER = "sora2.uclab.jp"
        self.MQTT_UCLAB_PORT = 1883

        self.MQTT_CTRL_TOPIC = "control/" + USER_UUID
        self.MQTT_TIME_TOPIC = "time/" + USER_UUID
        self.MQTT_SHARE_TOPIC = "share/" + USER_UUID
        self.MQTT_ROBOT_STATE_TOPIC = "robot/" + USER_UUID

        self.time_offset = 0
        self.time_vr = 0

        self.pose = np.zeros(16)

        self.shared_signal = 0 # shared_control_signal
        self.shared_control_flag = 0

    def on_connect(self, client, userdata, flags, rc):
        print("MQTT:Connected with result code " + str(rc), "subscribe topic:",
              self.MQTT_TIME_TOPIC, self.MQTT_CTRL_TOPIC, self.MQTT_SHARE_TOPIC)

        self.client.subscribe(self.MQTT_TIME_TOPIC)  # time
        self.client.subscribe(self.MQTT_CTRL_TOPIC)  # control
        self.client.subscribe(self.MQTT_SHARE_TOPIC) # shared control

        # ここで、MyID Register すべき
        my_info = {
            "date": str(datetime.today()),
            "version": "0.0.1",
            "devType": "robot",
            "robotModel": ROBOT_MODEL,
            "codeType": "PiPER-control",
            "devId": ROBOT_UUID
        }
        self.client.publish("mgr/register", json.dumps(my_info))
        print("Publish", json.dumps(my_info))

        # self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)  # connected -> subscribe

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection.")

    def on_message(self, client, userdata, msg):
        if msg.topic == MQTT_MANAGE_RCV_TOPIC:  # 受信先を指定された場合
            js = json.loads(msg.payload)
            if "controller" in js:
                if "devId" in js:
                    self.MQTT_CTRL_TOPIC = "control/" + js["devId"]
                    self.client.subscribe(self.MQTT_CTRL_TOPIC)
                    print("Receive Teleoperation Control msg, then listen", self.MQTT_CTRL_TOPIC)

                    self.MQTT_TIME_TOPIC = "time/" + js["devId"]
                    self.client.subscribe(self.MQTT_TIME_TOPIC)
                    print("Receive Time msg, then listen", self.MQTT_TIME_TOPIC)

                    self.MQTT_SHARE_TOPIC = "share/" + js["devId"]
                    self.client.subscribe(self.MQTT_SHARE_TOPIC)
                    print("Receive Shared Control msg, then listen", self.MQTT_SHARE_TOPIC)

        if msg.topic == self.MQTT_CTRL_TOPIC:
            # Message ThetaBody
            js_joints = json.loads(msg.payload)
            joints = js_joints[self.joint_topic]
            thetaBody = [joints[i] if i < len(joints) else 0 for i in range(7)]
            # print("Set thetaBody:", thetaBody)

            # Message ThetaTool
            js_tool = js_joints[self.tool_topic]
            thetaTool = js_tool

            # Save to shared memory
            self.pose[8:15] = thetaBody
            self.pose[15] = thetaTool

            self.time_vr = js_joints["timestamp"]

        elif msg.topic == self.MQTT_TIME_TOPIC:
            js_time = json.loads(msg.payload)
            self.time_offset = js_time["time_offset"]
            # print("Time Offset:", self.time_offset)

        elif msg.topic == self.MQTT_SHARE_TOPIC:
            js_share = json.loads(msg.payload)
            self.shared_control_flag = js_share["flag"]
            self.shared_signal = js_share["share"]

            # print("Shared Control Signal:", self.shared_signal)

        else:
            print("not subscribe msg", msg.topic)

    def start_mqtt(self):
        if self.mode == "local":
            self.client.tls_set(cert_reqs=0)
            self.client.on_connect = self.on_connect
            self.client.on_disconnect = self.on_disconnect
            self.client.on_message = self.on_message
            self.client.connect(self.MQTT_LOCAL_SERVER, self.MQTT_LOCAL_PORT, 60)
            self.client.loop_start()

        elif self.mode == "uclab":
            self.client.on_connect = self.on_connect
            self.client.on_disconnect = self.on_disconnect
            self.client.on_message = self.on_message
            self.client.connect(self.MQTT_UCLAB_SERVER, self.MQTT_UCLAB_PORT, 60)
            self.client.loop_start()

    def create_shared_memory(self, name_shared_memory):
        try:
            self.sm = sm.SharedMemory(name_shared_memory, create=True, size=16 * 4)
            self.pose = np.ndarray((16,), dtype=np.float32, buffer=self.sm.buf)
            self.pose[:] = 0
            print(f"Shared memory {name_shared_memory} created.")
        except FileExistsError:
            self.sm = sm.SharedMemory(name_shared_memory)
            self.pose = np.ndarray((16,), dtype=np.float32, buffer=self.sm.buf)
            print(f"Shared memory {name_shared_memory} already exists.")

    def write_shared_memory(self, name_shared_memory, data=None):
        try:
            # Connect to an existing shared memory block
            existing_sm = sm.SharedMemory(name=name_shared_memory)
            # Create a numpy view of the buffer
            pose = np.ndarray((16,), dtype=np.float32, buffer=existing_sm.buf)

            if data is not None:
                # Ensure data matches shape and dtype
                data = np.array(data, dtype=np.float32)
                if data.shape != pose.shape:
                    raise ValueError(f"Shape mismatch: expected {pose.shape}, got {data.shape}")

                # Write new values into shared memory (directly modifies memory)
                pose[:] = data[:]
                # print("Wrote data to shared memory:", pose[:])
            else:
                print("Current shared memory content:", pose[:])

        except FileNotFoundError:
            print(f"Shared memory {name_shared_memory} not found.")
        finally:
            try:
                existing_sm.close()
            except:
                pass

    def read_shared_memory(self, name_shared_memory):
        try:
            # Connect to an existing shared memory block
            existing_sm = sm.SharedMemory(name=name_shared_memory)
            # Create a numpy view of the shared buffer
            pose = np.ndarray((16,), dtype=np.float32, buffer=existing_sm.buf)
            # print("Read values from shared memory:", pose[:])

            # Optionally, do something with pose...
            return pose.copy()  # copy if you want to avoid changes when buffer updates
        except FileNotFoundError:
            print(f"Shared memory {name_shared_memory} not found.")
            return None
        finally:
            # Close handle (does NOT delete the shared memory)
            try:
                existing_sm.close()
            except:
                pass

    def verify_shared_memory(self, name, expected_shape=(16,), dtype=np.float32):
        try:
            shm = sm.SharedMemory(name=name)
            print("shm", shm )
            arr = np.ndarray(expected_shape, dtype=dtype, buffer=shm.buf)
            # test
            _ = arr[0]
            shm.close()
            print(f"✅ Shared Memory {name} available.")
            return True
        except (FileNotFoundError, ValueError, BufferError):
            print(f"❌ Shared Memory {name} not exist or unavailable.")
            return False

    def close_shared_memory(self, name_shm):
        shm = sm.SharedMemory(name=name_shm)
        shm.close()
        print(f"Shared Memory {name_shm} closed.")
        return

    def get_shared_memory(self):
        return self.pose

    def get_shm_object(self):
        return self.sm

    def publish_message(self, payload_dict):
        self.client.publish(self.MQTT_ROBOT_STATE_TOPIC, json.dumps(payload_dict))

    def get_time_offset(self):
        return self.time_offset

    def get_time_vr(self):
        return self.time_vr

    def get_shared_control_signal(self):
        return self.shared_signal

    def get_shared_control_flag(self):
        return self.shared_control_flag