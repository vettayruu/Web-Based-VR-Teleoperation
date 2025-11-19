import React from 'react';
import Assets from './Assets';
import { Select_Robot } from './Model';
import WebInterface from './web_interface.js';

export default function RobotScene(props) {
  const {
    robot_list, 
    robotProps, 
    
    // VR
    rendered, 
    interfacePropos, 
    view_cam_pose,

    // Right Arm
    state_codes, 
    position_ee, 
    euler_ee, 
    vr_controller_pos, 
    vr_controller_R,
    rightArmPosition,

    // Left Arm
    state_codes_left,
    position_ee_left, 
    euler_ee_left, 
    vr_controller_pos_left, 
    vr_controller_R_left,
    leftArmPosition,

    // Cam Arm
    state_codes_cam,
    position_ee_cam,
    euler_ee_cam,

    // Others
    // modelOpacity, 
    // webcamStream1, 
    // webcamStream2,
    // dsp_message,
    showMenu,
  } = props;

  const getStateCodeColor = (code) => {
    const colorMap = {
      0x00: "yellow",    // NORMAL
      0x01: "red",       // IK_FAILED  
      0x02: "orange",    // VELOCITY_LIMIT
      0x03: "purple",    // JOINT_LIMIT
      0x04: "pink",      // SINGULARITY
      0x05: "gray",      // VR_INPUT_INVALID
      0x06: "blue",      // JACOBIAN_ERROR
      0x07: "cyan",      // TARGET_UNREACHABLE
    };
    return colorMap[code] || "white";
  };

  const stateCodeColor = getStateCodeColor(state_codes);
  const stateCodeColorLeft = getStateCodeColor(state_codes_left);
  const stateCodeColorCam = getStateCodeColor(state_codes_cam);

  const rad2deg = rad => rad * 180 / Math.PI;
  const euler_ee_deg = euler_ee.map(rad2deg);
  const euler_ee_deg_left = euler_ee_left.map(rad2deg);
  const euler_ee_deg_cam = euler_ee_cam.map(rad2deg);

  const now = Date.now();
  const [activeButton, setActiveButton] = React.useState(null);
  
  const scale = 0.1;
  const xAxis = {
    x: vr_controller_R[0][0] * scale,
    y: vr_controller_R[1][0] * scale,
    z: vr_controller_R[2][0] * scale,
  };
  const yAxis = {
    x: vr_controller_R[0][1] * scale,
    y: vr_controller_R[1][1] * scale,
    z: vr_controller_R[2][1] * scale,
  };
  const zAxis = {
    x: vr_controller_R[0][2] * scale,
    y: vr_controller_R[1][2] * scale,
    z: vr_controller_R[2][2] * scale,
  };

  const xAxis_left = {
    x: vr_controller_R_left[0][0] * scale,
    y: vr_controller_R_left[1][0] * scale,
    z: vr_controller_R_left[2][0] * scale,
  };
  const yAxis_left = {
    x: vr_controller_R_left[0][1] * scale,
    y: vr_controller_R_left[1][1] * scale,
    z: vr_controller_R_left[2][1] * scale,
  };
  const zAxis_left = {
    x: vr_controller_R_left[0][2] * scale,
    y: vr_controller_R_left[1][2] * scale,
    z: vr_controller_R_left[2][2] * scale,
  };

  // Webcam Stream
  React.useEffect(() => {
    if (props.webcamStream1) {
      const videoEl = document.getElementById('leftVideo');
      if (videoEl && videoEl.srcObject !== props.webcamStream1) {
        videoEl.srcObject = props.webcamStream1;
        videoEl.play();
      }
    }
  }, [now]);

  React.useEffect(() => {
    if (props.webcamStream2) {
      const videoEl = document.getElementById('rightVideo');
      if (videoEl && videoEl.srcObject !== props.webcamStream2) {
        videoEl.srcObject = props.webcamStream2;
        videoEl.play();
      }
    }
  }, [now]);

  React.useEffect(() => {
    if (props.webcamStream3) {
      const videoEl = document.getElementById('subVideo');
      if (videoEl && videoEl.srcObject !== props.webcamStream3) {
        videoEl.srcObject = props.webcamStream3;
        videoEl.play();
      }
    }
  }, [now]);


  if (!rendered) {
    return (
      <a-scene xr-mode-ui="XRMode: ar">
        <Assets robot_list={robot_list} viewer={props.viewer}/>
      </a-scene>
    );
    }

  return (
    <>
      <a-scene scene xr-mode-ui="XRMode: ar">
        {showMenu && (<a-entity
          id="background"
          position="0 0 0"
          geometry="primitive: sphere; radius: 2.0"
          material="color: gray; side: back; shader: flat"
          scale="0.001 0.001 0.001"
          visible="true" class="raycastable">
        </a-entity>)}

        {showMenu && (
          <a-entity id="menu" position="0 1.0 -1" highlight button-action>
            {/* Background Plane */}
            <a-plane
              width="1.2"
              height="1.2"
              color="#222"
              opacity="1.0"
              position="0 0 0"
              class="raycastable"
            ></a-plane><a-text value="Menu" align="center" color="#fff" width="2.0" position="0 0.4 0.01"></a-text>
            {/* Button 1 */}
            <a-entity id="button1" position="-0.3 0.2 0.01" class="raycastable menu-button"
              geometry="primitive: plane; width: 0.4; height: 0.18"
              material="color: white; opacity: 0.95"
            ><a-text value="Control Mode \n inSpace" align="center" color="#fff" width="1.0" position="0 0 0.01"></a-text></a-entity>
            
            {/* Button 2 */}
            <a-entity id="button2" position="0.3 0.2 0.01" class="raycastable menu-button"
              geometry="primitive: plane; width: 0.4; height: 0.18"
              material="color: white; opacity: 0.95"
            ><a-text value="Control Mode \n inBody" align="center" color="#fff" width="1.0" position="0 0 0.01"></a-text></a-entity>
            
            {/* Button 3 */}
            <a-entity id="button3" position="-0.3 0.0 0.01" class="raycastable menu-button"
              geometry="primitive: plane; width: 0.4; height: 0.18"
              material="color: white; opacity: 0.95"
            ><a-text value="Dual Arm Control \n Off" align="center" color="#fff" width="1.0" position="0 0 0.01"></a-text></a-entity>

            {/* Button 4 */}
            <a-entity id="button4" position="0.3 0.0 0.01" class="raycastable menu-button"
              geometry="primitive: plane; width: 0.4; height: 0.18"
              material="color: white; opacity: 0.95"
            ><a-text value="Dual Arm Control \n On" align="center" color="#fff" width="1.0" position="0 0 0.01"></a-text></a-entity>

            {/* Button 5 */}
            <a-entity id="button5" position="-0.3 -0.2 0.01" class="raycastable menu-button"
              geometry="primitive: plane; width: 0.4; height: 0.18"
              material="color: white; opacity: 0.95"
            ><a-text value="Indicator \n On" align="center" color="#fff" width="1.0" position="0 0 0.01"></a-text></a-entity>

            {/* Button 6 */}
            <a-entity id="button6" position="0.3 -0.2 0.01" class="raycastable menu-button"
              geometry="primitive: plane; width: 0.4; height: 0.18"
              material="color: white; opacity: 0.95"
            ><a-text value="Indicator \n Off" align="center" color="#fff" width="1.0" position="0 0 0.01"></a-text></a-entity>

            {/* Button 7 */}
            <a-entity id="button7" position="-0.3 -0.4 0.01" class="raycastable menu-button"
              geometry="primitive: plane; width: 0.4; height: 0.18"
              material="color: white; opacity: 0.95"
            ><a-text value="Visual Assist \n On" align="center" color="#fff" width="1.0" position="0 0 0.01"></a-text></a-entity>

            {/* Button 8 */}
            <a-entity id="button8" position="0.3 -0.4 0.01" class="raycastable menu-button"
              geometry="primitive: plane; width: 0.4; height: 0.18"
              material="color: white; opacity: 0.95"
            ><a-text value="Visual Assist \n Off" align="center" color="#fff" width="1.0" position="0 0 0.01"></a-text></a-entity>

          </a-entity>
        )}

        {showMenu && (<a-entity id="leftHand" laser-controls="hand: left" raycaster="objects: .raycastable"></a-entity>)}
        {showMenu && (<a-entity id="rightHand" laser-controls="hand: right" raycaster="objects: .raycastable" line="color: #118A7E"></a-entity>)}

        {/* VR Controller */}
        <a-entity oculus-touch-controls="hand: right" vr-controller-right visible={true} opacity={0.5}></a-entity>
        <a-entity oculus-touch-controls="hand: left" vr-controller-left visible={true} opacity={0.5}></a-entity>

        {/* Robot Model*/}
        <Assets robot_list={robot_list} viewer={props.viewer} monitor={props.monitor}/>
        <Select_Robot 
          {...robotProps} 
          modelOpacity={props.modelOpacity}
          position_left={leftArmPosition}
          position_right={rightArmPosition}
          indicator_visibility={props.indicator}
        />

        {/* Remote Cam*/}
        <a-assets>
          <video id="leftVideo" autoPlay playsInline crossOrigin="anonymous" muted></video>
          <video id="rightVideo" autoPlay playsInline crossOrigin="anonymous" muted></video>
          <video id="subVideo" autoPlay playsInline crossOrigin="anonymous" muted></video>
        </a-assets>

        {/* Plane or curved image for pinhole camera (ZED camera etc.) */}
        {/* Curved Image For 720P camera frame (Before undistortion) */}
        {/* <a-curvedimage
          id="left-curved"
          height="7.0"
          radius="5.7"
          theta-length="120"
          position="0.2 1.6 -1.0"
          rotation="0 -115 0"
          scale="-1 1 1"
          stereo-curvedvideo="eye: left; videoId: leftVideo">
        </a-curvedimage>

        <a-curvedimage
          id="right-curved"
          height="7.0"
          radius="5.7"
          theta-length="120"
          position="0.2 1.6 -1.0"
          rotation="0 -120.3 0"
          scale="-1 1 1"
          stereo-curvedvideo="eye: right; videoId: rightVideo">
        </a-curvedimage> */}

        {/* For 1080P */}
        {/* <a-curvedimage
          id="left-curved"
          height="9.0"
          radius="5.7"
          theta-length="180"
          position="-0.30 1.2 -1.50"
          rotation="0 -115 0"
          scale="-1 1 1"
          stereo-curvedvideo="eye: left; videoId: leftVideo"
          visible="true"
        ></a-curvedimage>

        <a-curvedimage
          id="right-curved"
          height="9.0"
          radius="5.7"
          theta-length="180"
          position="0.20. 1.2 -1.50"
          rotation="0 -121 0"
          scale="-1 1 1"
          stereo-curvedvideo="eye: right; videoId: rightVideo"
          visible="true"
        ></a-curvedimage> */}

        {/* Plane Image For 720P camera frame (After undistortion)*/}
        <a-plane
          id="left-curved"
          height="4.0"
          width="7.0"
          position="-0.16 1.0 -2.3" // 1080p position="-0.19 1.0 -2.3"; 720p position="0.16 1.0 -2.3"
          scale="0.6 0.6 0.2"
          stereo-plane="eye: left; videoId: leftVideo"
          visible="true"
        ></a-plane>

        <a-plane
          id="right-curved"
          height="4.0"
          width="7.0"
          position="0.16 1.0 -2.3" // 1080p position="0.19 1.0 -2.3"; 720p position="0.16 1.0 -2.3"
          scale="0.6 0.6 0.2"
          stereo-plane="eye: right; videoId: rightVideo"
          visible="true"
        ></a-plane>

        {/* Sub Camera Plane Image */}
        <a-plane
          id="subcam-curved"
          height="1.5"
          width="2.3"
          position="1.1 0.0 -2.0" // 1080p position="0.19 1.0 -2.3"; 720p position="0.16 1.0 -2.3"
          scale="0.6 0.6 0.2"
          stereo-plane="eye: left; videoId: subVideo"
          visible="true"
        ></a-plane>

        <a-plane
          id="subcam-curved"
          height="1.5"
          width="2.3"
          position="1.1 0.0 -2.0" // 1080p position="0.19 1.0 -2.3"; 720p position="0.16 1.0 -2.3"
          scale="0.6 0.6 0.2"
          stereo-plane="eye: right; videoId: subVideo"
          visible="true"
        ></a-plane>

        {/* Sphere video for fisheye camera (VR cam etc.)*/}
        {/* <a-sphere
          id="left-curved"
          // radius="800"
          position="-0.28 1.0 -0.5"
          scale="-0.05 0.062 0.015"
          stereo-spherevideo="eye: left; videoId: leftVideo"
          geometry="primitive: sphere; radius: 8; thetaStart: 0; thetaLength: 180"
        ></a-sphere>

        <a-sphere
          id="right-curved"
          // radius="8"
          position="0.28 1.0 -0.5"
          scale="-0.05 0.062 0.015"
          stereo-spherevideo="eye: right; videoId: rightVideo"
          geometry="primitive: sphere; radius: 8; thetaStart: 0; thetaLength: 180"
        ></a-sphere> */}
        
        
        {/* Light */}
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="1 1 1"></a-entity>
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="-1 1 1"></a-entity>
        <a-entity light="type: directional; color: #EEE; intensity: 0.25" position="-1 1 -1"></a-entity>
        <a-entity light="type: directional; color: #FFF; intensity: 0.25" position="1 1 -1"></a-entity>
        <a-entity light="type: directional; color: #EFE; intensity: 0.1" position="0 -1 0"></a-entity>
        <a-entity id="rig" position={`${view_cam_pose[0]} ${view_cam_pose[1]} ${view_cam_pose[2]}`} rotation={`${view_cam_pose[3]} ${view_cam_pose[4]} ${view_cam_pose[5]}`}>

          {/* Camera */}
          <a-camera id="camera" cursor="rayOrigin: mouse;" position="0 0 0">
            {/* <a-entity jtext={`text: ${dsp_message}; color: black; background:rgb(31, 219, 255); border: #000000`} position="0 0.7 -1.4"></a-entity>
            <a-curvedimage
              id="left-curved"
              height="9.0"
              radius="5.7"
              theta-length="155"
              position="-0.2 0 0"
              rotation="0 -115 0"
              scale="-1 1 1"
              stereo-curvedvideo="eye: left; videoId: leftVideo"
            ></a-curvedimage>

            <a-curvedimage
              id="right-curved"
              height="9.0"
              radius="5.7"
              theta-length="155"
              position="-0.2 0 0"
              rotation="0 -121 0"
              scale="-1 1 1"
              stereo-curvedvideo="eye: right; videoId: rightVideo"
            ></a-curvedimage> */}
          </a-camera>

          {/* End Effector Right*/}
          </a-entity>
          <a-sphere 
            position={`${position_ee[0]+0.3} ${position_ee[1]} ${position_ee[2]}`} 
            scale="0.012 0.012 0.012" 
            color={stateCodeColor}
            visible={true}></a-sphere>
          <a-entity
            position={`${position_ee[0]+0.3} ${position_ee[1]} ${position_ee[2]}`}
            // ZYX
            rotation={`${euler_ee_deg[0]} ${-euler_ee_deg[2]} ${-euler_ee_deg[1]} `}
          >
            <a-cylinder position="0      0     -0.015" rotation="90 0  0 " height="0.0500" radius="0.0015" color="red" /> 
            <a-cylinder position="-0.015      0     0" rotation="0  0  90" height="0.0500" radius="0.0015" color="green" />
            <a-cylinder position="0      0.025      0" rotation="0  90 0 " height="0.0700" radius="0.0015" color="blue" />
          </a-entity>

          {/* End Effector Right Copy*/}
          <a-entity
            position={`${vr_controller_pos[0]} ${vr_controller_pos[1]} ${vr_controller_pos[2]}`} 
            // ZYX
            rotation={`${euler_ee_deg[0]} ${-euler_ee_deg[2]} ${-euler_ee_deg[1]} `}
            >
            <a-cylinder position="0      0     -0.05" rotation="90 0  0 " height="0.150" radius="0.0035" color="red" /> 
            <a-cylinder position="-0.05      0     0" rotation="0  0  90" height="0.150" radius="0.0035" color="green" />
            <a-cylinder position="0      0.05      0" rotation="0  90 0 " height="0.150" radius="0.0035" color="blue" />
          </a-entity>

          {/* End Effector Left */}
          <a-sphere 
            position={`${position_ee_left[0]-0.3} ${position_ee_left[1]} ${position_ee_left[2]}`} 
            scale="0.012 0.012 0.012" 
            color={stateCodeColorLeft}
            visible={true}></a-sphere>
          <a-entity
            position={`${position_ee_left[0]-0.3} ${position_ee_left[1]} ${position_ee_left[2]}`}
            // ZYX
            rotation={`${euler_ee_deg_left[0]} ${-euler_ee_deg_left[2]} ${-euler_ee_deg_left[1]} `}
            >
            <a-cylinder position="0      0     -0.015" rotation="90 0  0 " height="0.0500" radius="0.0015" color="red" /> 
            <a-cylinder position="-0.015      0     0" rotation="0  0  90" height="0.0500" radius="0.0015" color="green" />
            <a-cylinder position="0      0.025      0" rotation="0  90 0 " height="0.0700" radius="0.0015" color="blue" />
          </a-entity>

          {/* End Effector Left Copy*/}
          <a-entity
            position={`${vr_controller_pos_left[0]} ${vr_controller_pos_left[1]} ${vr_controller_pos_left[2]}`} 
            // ZYX
            rotation={`${euler_ee_deg_left[0]} ${-euler_ee_deg_left[2]} ${-euler_ee_deg_left[1]} `}
            >
            <a-cylinder position="0      0     -0.05" rotation="90 0  0 " height="0.150" radius="0.0035" color="red" /> 
            <a-cylinder position="-0.05      0     0" rotation="0  0  90" height="0.150" radius="0.0035" color="green" />
            <a-cylinder position="0      0.05      0" rotation="0  90 0 " height="0.150" radius="0.0035" color="blue" />
          </a-entity>

          {/* End Effector Cam */}
          <a-sphere 
            position={`${position_ee_cam[0]} ${position_ee_cam[1]+0.208} ${position_ee_cam[2]+0.035}`} 
            scale="0.012 0.012 0.012" 
            color={stateCodeColorCam}
            visible={true}></a-sphere>
          <a-entity
            position={`${position_ee_cam[0]} ${position_ee_cam[1]+0.208} ${position_ee_cam[2]+0.035}`}
            // ZYX
            rotation={`${euler_ee_deg_cam[0]} ${-euler_ee_deg_cam[2]} ${-euler_ee_deg_cam[1]} `}
          >

            {/* ZYX */}
            <a-cylinder position="0      0     -0.015" rotation="90 0  0 " height="0.0500" radius="0.0015" color="red" /> 
            <a-cylinder position="-0.015      0     0" rotation="0  0  90" height="0.0500" radius="0.0015" color="green" />
            <a-cylinder position="0      0.025      0" rotation="0  90 0 " height="0.0700" radius="0.0015" color="blue" />
          </a-entity>

          {/* VR Controller Pose Right*/}
          <a-entity
            position={`${vr_controller_pos[0]} ${vr_controller_pos[1]} ${vr_controller_pos[2]}`} 
          >
            <a-entity
              line={`start: 0 0 0; end: ${xAxis.x} ${xAxis.y} ${xAxis.z}; color: red;`}
              visible="true"
              opacity="0.3"
            />
            <a-entity
              line={`start: 0 0 0; end: ${yAxis.x} ${yAxis.y} ${yAxis.z}; color: green;`}
              visible="true"
              opacity="0.3"
            />
            <a-entity
              line={`start: 0 0 0; end: ${zAxis.x} ${zAxis.y} ${zAxis.z}; color: blue;`}
              visible="true"
              opacity="0.3"
            />
          </a-entity>

          {/* VR Controller Pose Left*/}
          <a-entity
            position={`${vr_controller_pos_left[0]} ${vr_controller_pos_left[1]} ${vr_controller_pos_left[2]}`} 
          >
            <a-entity
              line={`start: 0 0 0; end: ${xAxis_left.x} ${xAxis_left.y} ${xAxis_left.z}; color: red;`}
              visible="true"
              opacity="0.3"
            />
            <a-entity
              line={`start: 0 0 0; end: ${yAxis_left.x} ${yAxis_left.y} ${yAxis_left.z}; color: green;`}
              visible="true"
              opacity="0.3"
            />
            <a-entity
              line={`start: 0 0 0; end: ${zAxis_left.x} ${zAxis_left.y} ${zAxis_left.z}; color: blue;`}
              visible="true"
              opacity="0.3"
            />
          </a-entity>
      </a-scene>

      <WebInterface {...interfacePropos}/>

    </>
  );
}