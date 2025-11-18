# Web-Based VR Teleoperation System

<div align="center">
  <img src="./system.svg" alt="System Architecture" width="800"/>
  <p><em>Figure 1: System Overview.</em></p>
</div>

## Quick Start
- [Step 1: Run HTTPS Server](#step-1-run-https-server)
- [Step 2: Build MQTT Broker](#step-2-build-mqtt-broker)
- [Step 3: Debug with WebXR and Simulator](#step-3-debug-with-webxr-and-simulator)
- [Simulator](#run-in-simulator)
- [VR Controller](#open-mqtt-controller-in-vr)
- [Controller Operations](#controller-operations)
- [Notifications](#notifications)
- [Citations](#citations)
- [Demo Videos](#demo-videos)
- [Python Packages](#python-packages)

---
## Step 1: Run HTTPS Server

💡 **If this is your first time running the project, install the required Node.js modules:**
```bash
npm install
```

**The project is designed to run in VS Code. Download it here:**  
[https://code.visualstudio.com/](https://code.visualstudio.com/Download)

**If you do not have Node.js installed, download it here:**  
[https://nodejs.org/en/download](https://nodejs.org/en/download)

**On Windows, you may need to allow script execution before running the server:**
```powershell
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
```

🚀 **To start the Next.js HTTPS server, run:**
```bash
npm run dev-https
```

After starting, you will see two URLs:
- Local:   [https://localhost:3000](https://localhost:3000)
- Network: [https://192.168.197.39:3000](https://192.168.197.39:3000)

> The Network IP address may vary depending on your network environment.

⚠️ **In VR, only HTTPS can enter VR/AR mode.**

**Open the browser in your VR device and enter `https://192.168.197.39:3000` to access the web interface.**

---
## Step 2: Build MQTT Broker

### 1. Install Mosquitto

Download Mosquitto from the official website: [https://mosquitto.org/download/](https://mosquitto.org/download/)

On Windows, Mosquitto is usually installed in:
```
C:\Program Files\mosquitto
```

### 2. Configure Mosquitto

Open `mosquitto.conf` in the installation folder and add the following settings:

**WebSocket (non-secure, ws):**
```
listener 9001
protocol websockets
```
This enables WebSocket connections on port 9001.

**Secure WebSocket (wss):**

First, generate `.pem` certificates.

To generate self-certification files, in the folder `MQTT_Client` run
```bash
node .\generate-ssl-cert.js 
```

Then you can get `cert.pem` and `key.pem` self-certification files.

Copy `cert.pem` and `key.pem` to the installation folder, add the following lines to `mosquitto.conf`:
```
listener 8333
protocol websockets
certfile C:\Program Files\mosquitto\cert.pem
keyfile C:\Program Files\mosquitto\key.pem
allow_anonymous true
```
Port numbers (e.g., 9001, 8333) can be customized.

### 3. Verify the MQTT Broker

**Find your server address:**  
On Windows, run `ipconfig` in the terminal, and look for:

```
IPv4 Address. . . . . . . . . . . : 192.168.197.39
```

Use this IP together with your MQTT port. For example:
```
192.168.197.39:9001   (for ws)
192.168.197.39:8333   (for wss)
```

> ⚠️ **Important: Verify the MQTT port before MQTT communication.**  
> To verify, open your browser and go to your MQTT port. For example:
> ```
> https://192.168.197.39:8333
> ```

### 4. Start the MQTT Broker

After verifying, you can use your local MQTT server for communication in your local network.

**Run Mosquitto as Administrator:**
```
cd "C:\Program Files\mosquitto"
mosquitto -v
```

### 5. Test Your MQTT Broker

To test publishing a topic, navigate to the `MQTT_Client` folder and run:

**Publish a test message:**
```bash
python local_mqtt_test_pub.py
```

**Subscribe to the test message:**
```bash
python local_mqtt_test_sub.py
```

To check all topics in MQTT, run:
```bash
python MQTT_Topic_list.py
```

---
## Step 3: Debug with WebXR and Simulator

If you don't have a VR device, you can use WebXR tools for debugging.

On Chrome, you can install the **Immersive Web Emulator** WebXR plugin:

[Immersive Web Emulator (Chrome Web Store)](https://chromewebstore.google.com/detail/immersive-web-emulator/cgffilbpcibhmcfbgggfhfolhkfbhmik)

After installation, press **F12** to open Developer Tools.  
You will find the **WebXR** tab in the developer tools bar, which allows you to emulate VR devices and test WebXR features directly in your browser.

### 🧪 Run in Simulator

You can also simulate teleoperation using [CoppeliaSim](https://www.coppeliarobotics.com/).

1. **Download CoppeliaSim**

   Visit the official website to download the latest version:

   ```
   https://www.coppeliarobotics.com/
   ```

2. **Launch CoppeliaSim**

   - **On Ubuntu:** Navigate to your CoppeliaSim installation directory and run:
     ```bash
     ./coppeliaSim
     ```
   - **On Windows:** Run the application directly by double-clicking the executable.

3. **Load the simulation scene**

   In CoppeliaSim, open the scene file `"piper_robot_sample.ttt"` located in the `Simulation` folder.

4. **Start the simulation**

   Click the "Play" button in CoppeliaSim to start the simulation.

5. **Run the simulator control script**
   ```bash
   python MQTT_Robot_Simulator.py
   ```

## Step 4: Run Your Robot

###  🧩 Step 2: Run PiPER Controller (Robot Site)
Follow the steps below to control the **AgileX-PiPER** robot via MQTT:

1. **Activate the CAN bus**
   Execute the following command in your terminal:
   ```bash
   cd ./AgileX-PiPER-MetworkMQTT
   bash can_activate.sh can0 1000000
   ```

2. **Start the PiPER SDK UI**
   
   (check "agilex_pipier_connect.mkv" in the video folder )
   
   🌐 Download the PiPER SDK UI
   
   ```arduion
   https://github.com/agilexrobotics/Piper_sdk_ui.git
   ```

   Open PiPER SDK UI
   ```bash
   cd Piper_sdk_ui
   python piper_ui.py 
   ```

   To reset the robot, open the PiPER SDK Tools and perform the following operations:

      (0) Click **Find CAN Port**
      
      (1) Click **Reset**
      
      (2) Click **Enable**
      
      (3) Click **Go Zero**
      
   🔁 If the robot fails to go to the zero position, repeat steps (1)~(3) a few times until successful.

   To reset the tool, Click **Gripper Zero**

4. **Set the robot to the working position**
   
   Run the following script:
   ```bash
   python piper_work_position_initialize.py
   ```

   ⚠️ If this is your first time using the robot, **calibration** may be required before running the script.

5. **Retrieve your USER_UUID**
   
   (check "mqtt_teleoperation_start.mp4" in the video folder )
   
   Open the Viewer in your browser:
   
   ```arduion
   https://<your-server-address>/viewer
   ```

   For example:
   ```arduion
   https://192.168.197.37:3000/viewer/
   ```
   
   Press F12 to open Developer Tools
   
   Look for the **USER_UUID** in the console or network tab and copy it to the "MQTT_Recv.py", Line 25. For example:

   ```python
   USER_UUID = "84f289d0-bf07-4ad2-baf1-a4c8f7c9a763-qk4b9zg-viewer"
   ```

   where
   - `84f289d0-bf07-4ad2-baf1-a4c8f7c9a763` is the device ID, and it changes with IP address
   -  `qk4b9zg` is a random string, and it changes with broswer tag
   -  The `USER_UUID` is generated by  `./lib/cookie_id.js`

    **Important**: **USER_UUID changes every time** the MQTT Controller (User Site) is restarted. You must repeat this step each time you launch the MQTT Controller to ensure proper functionality.

    **Option**: In `./lib/cookie_id.js`, line 53, the variable `name` is used to generate the cookie identifier. However, this causes a **new number to be generated whenever the browser changes**, leading to inconsistent user identification.
    To make the identifier **static across sessions or browsers**, a **customized static name** (e.g., `USER_UUID`) should be used instead.
   
8. **Run the Robot Controller Script**

   Choose one of the following options depending on your control needs:

   - **PD Control + Trajectory Planning**  
     This is the most stable option, as both velocity and acceleration are smoothly planned.
     ```bash
     python MQTT_Robot_Feedback_PD_Traj.py
     ```

   - **PD Control**  
     Basic proportional-derivative control without trajectory planning.
     ```bash
     python MQTT_Robot_Feedback_PD.py
     ```

   - **Direct Control Signal**  
     Sends raw control signals directly to the robot without any feedback or planning.
     ```bash
     python MQTT_Robot_Control.py
     ```



## 🕶️ Open MQTT Controller in VR

To operate the robot in VR, open the controller interface in the browser inside your VR headset:

  ```arduion
  https://<your-server-address>
  ```

  For example:
  ```arduion
  https://192.168.197.37:3000
  ```

  Once the page is open, press the "AR" button to enter augmented reality mode.
   
### 🎮 Controller Operations

The following input mappings are used to operate the PiPER robot via the VR controller:

| Input Combination         | Action       |
|---------------------------|--------------|
| Trigger                   | Move the robot (6-DoF pose) |
| Button A           | Grasp   |
| Button B           | Release  |

> Make sure the controller is tracked and visible to the VR camera to ensure accurate input.


### ⚠️ Notifications

1. **Keep the VR controller within camera view**  
   The pose of the VR controller is estimated using both the onboard **accelerometer** and the **tracking camera** located on the side of the VR headset.  
   > ⚠️ If the controller goes out of view, pose estimation may become inaccurate, resulting in input drift.

2. **Wait for system initialization**  
   After putting on the VR headset or restarting the system, **always wait until initialization is complete**.  
   Skipping this step may result in control drift or unstable input.
   
   > ⚠️ If the controller appears frozen or unresponsive, it may indicate a tracking issue.
   
   > ✅ If you can see the controller moving in sync with your hand, it is functioning correctly.  

## 📚 Citations

The inverse kinematics (IK) implementation in this project is based on the **Modern Robotics** library by Kevin Lynch et al.

- 📘 Book: *Modern Robotics: Mechanics, Planning, and Control*  
- 💻 Source Code: [NxRLab/ModernRobotics GitHub Repository](https://github.com/NxRLab/ModernRobotics)

## 📹 Demo Videos

- [▶️ Demo 1: Cloth Folding 1](https://youtu.be/y29keqx_X6Q)
- [▶️ Demo 2: Cloth Folding 2](https://youtu.be/i-OcnSqnyN8)

## 🐍 Python Packages

It is recommended to use a `conda` environment to manage dependencies, for example:

```bash
conda create -n Modern_Robotics_Control_IK python=3.12
conda activate Modern_Robotics_Control_IK
```

If you forgot your conda environment name:
```bash
conda info --envs
```
to check your conda environemnt list.

### Dependencies
📐 Math & Utilities
```bash
pip install numpy
```

📡 Communication
```bash
pip install paho-mqtt
pip install python-dotenv
pip install ipget
```

🤖 Robotics
```bash
pip install piper-sdk
pip install modern-robotics
```

🖥️ Robot Monitor UI
```bash
pip install PyQt5
pip install pyqtgraph
```

🧪 Simulator (CoppeliaSim Remote API)
```bash
pip install coppeliasim-zmqremoteapi-client
```


