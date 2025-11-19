import paho.mqtt.client as mqtt

broker = "192.168.197.29"
port = 8333
# topic = "test/ws"
topic = "control/be83bf27-4c4a-4d51-975a-d7bd3ad830df-1v9yn3n"
topic_request = "mgr/request"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ Connected to broker")
        client.subscribe(topic)
        print(f"📡 Subscribed to {topic}")
    else:
        print("❌ Connection failed with code", rc)

def on_message(client, userdata, msg):
    print(f"📩 Received: {msg.payload.decode()} on {msg.topic}")

def on_topic_list(client, userdata, flags, rc):
    print('Connected with result code', rc)
    client.subscribe(topic_request)

client = mqtt.Client(transport="websockets")
client.tls_set(cert_reqs=0)
# client.on_connect = on_connect
client.on_message = on_message
client.on_connect = on_topic_list

print(f"Connecting to wss://{broker}:{port} ...")
client.connect(broker, port, 60)
client.loop_forever()

# 📩 Received: {"date":"2025/9/3 11:01:09","device":{"agent":"Mozilla/5.0 (X11; Linux x86_64; Quest 3) AppleWebKit/537.36 (KHTML, like Gecko) OculusBrowser/40.1.0.6.64.776937802 Chrome/138.0.7204.179 VR Safari/537.36","cookie":true},"devType":"browser","codeType":"PiPER-control-LIU","version":"0.1.1","devId":"d14ad41a-e958-4875-8528-ee3b5ffb73d2-at15uir"} on mgr/register
# 📩 Received: {"devId":"d14ad41a-e958-4875-8528-ee3b5ffb73d2-at15uir","type":"PiPER-control-LIU"} on mgr/request