"use client";
import * as React from 'react'
import "./web_interface.css";
import { idtopic } from '../lib/MetaworkMQTT'
import { soraConfig } from '../lib/WebRTC_Sora';

function rad2deg(rad) {
    if (Array.isArray(rad)) {
        return rad.map(r => rad2deg(r));
    }
    return rad * (180 / Math.PI);
}

function deg2rad(deg) {
    if (Array.isArray(deg)) {
        return deg.map(d => deg2rad(d));
    }
    return deg * (Math.PI / 180);
}

const channelId = 'wss://sora2.uclab.jp/signaling';

export default function WebInterface(props) {
  const {view_cam_pose} = props
  const {vr_mode} = props
  const {theta_body} = props
  const {theta_tool} = props
  const {position_ee} = props
  const {euler_ee} = props

  const {theta_body_left} = props
  const {theta_tool_left} = props
  const {theta_body_cam} = props

  const setViewCamPose = (index) => (e) => {
    let value = Number.parseFloat(e.target.value || 0);
    const newPose = [...view_cam_pose];
    newPose[index] = value;
    props.setViewCamPose(newPose);
  }

  const jointLimits = [
    { min: -45,  max: 45  },   // theta_1
    { min: -45,  max: 55  },   // theta_2
    { min: 45,   max: 145 },   // theta_3
    { min: -95,  max: 95  },   // theta_4
    { min: 1,    max: 69  },   // theta_5
    { min: -120, max: 120 },   // theta_6
    { min: -1, max: 89 }  // theta_tool
  ];

  const jointLimitsCam = [
    { min: -168, max: 168 },   // theta_1
    { min: -135, max: 135 },   // theta_2
    { min: -150, max: 150 },   // theta_3
    { min: -145, max: 145 },   // theta_4
    { min: -165, max: 165 },   // theta_5
    { min: -180, max: 180 },   // theta_6
    ];

  return (
    <>
      <div className="user-uuid">
        MQTT Control ID: <span>{idtopic}</span>
      </div>

      <div className="webrtc-channel">
        WebRTC Signaling Url: <span>{soraConfig.signalingUrl}</span><br/>
        Recv Channel 1 (Stereo Left): <span>{soraConfig.recv_channel_1}</span><br/>
        Recv Channel 2 (Stereo Right): <span>{soraConfig.recv_channel_2}</span><br/>
        Recv Channel 3 (Sub Cam): <span>{soraConfig.recv_channel_3}</span><br/>
        Send Channel 1 (VR Left): <span>{soraConfig.send_channel_1}</span><br/>
        Send Channel 2 (VR Right): <span>{soraConfig.send_channel_2}</span><br/>
      </div>


      <div className="view-cam" >
        {vr_mode?null:<><span>View Cam Pose</span>
        <div className="row mb-0">
          <div className="col-md-4"><label htmlFor="c_pos_x_number" className="form-label"><span className="form-control-plaintext">Left/Right</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_pos_x_number" value={view_cam_pose[0]} onChange={setViewCamPose(0)} step={0.01}/></div>
        </div>
        <div className="row mb-0">
          <div className="col-md-4"><label htmlFor="c_pos_y_number" className="form-label"><span className="form-control-plaintext">Up/Down</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_pos_y_number" value={view_cam_pose[1]} onChange={setViewCamPose(1)} step={0.01}/></div>
        </div>
        <div className="row mb-2">
          <div className="col-md-4"><label htmlFor="c_pos_z_number" className="form-label"><span className="form-control-plaintext">Fw/Back</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_pos_z_number" value={view_cam_pose[2]} onChange={setViewCamPose(2)} step={0.01}/></div>
        </div>
        <div className="row mb-0">
          <div className="col-md-4"><label htmlFor="c_deg_x_number" className="form-label"><span className="form-control-plaintext">↕️Pitch</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_deg_x_number" value={view_cam_pose[3]} onChange={setViewCamPose(3)} step={0.1}/></div>
        </div>
        <div className="row mb-0">
          <div className="col-md-4"><label htmlFor="c_deg_y_number" className="form-label"><span className="form-control-plaintext">↔️Yaw</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_deg_y_number" value={view_cam_pose[4]} onChange={setViewCamPose(4)} step={0.1}/></div>
        </div>
        <div className="row mb-2">
          <div className="col-md-4"><label htmlFor="c_deg_z_number" className="form-label"><span className="form-control-plaintext">🔄Roll</span></label></div>
          <div className="col-md-8"><input type="number" className="form-control" id="c_deg_z_number" value={view_cam_pose[5]} onChange={setViewCamPose(5)} step={0.1}/></div>
        </div>
        <div className="row mb-2">
        </div></>}
      </div>
      
      <div className="right-arm">
        <span>Right Arm</span>
        <div className="joint controller-controll-panel row">
          {theta_body.map((theta, idx) => (
            <div className="row mb-0" key={idx}>
              <div className="col-md-4">
                <label htmlFor={`theta_${idx+1}`} className="form-label">
                  <span className="form-control-plaintext">{`theta_${idx+1}`}</span>
                </label>
              </div>
              <div className="col-md-8">
                <input
                  type="number"
                  className="form-control"
                  id={`theta_${idx+1}`}
                  value={rad2deg(theta).toFixed(2)}
                  onChange={e => {
                    const degValue = Number.parseFloat(e.target.value || 0);
                    const newTheta = [...theta_body];
                    newTheta[idx] = deg2rad(degValue);
                    props.setThetaBody(newTheta);
                  }}
                  step={0.5}
                  min={jointLimits[idx].min}
                  max={jointLimits[idx].max}
                />
              </div>
            </div>
          ))}
          <div className="row mb-0">
            <div className="col-md-4">
              <label htmlFor="theta_tool" className="form-label">
                <span className="form-control-plaintext">theta_tool</span>
              </label>
            </div>
            <div className="col-md-8">
              <input
                type="number"
                className="form-control"
                id="theta_tool"
                value={(theta_tool).toFixed(2)}
                onChange={e => {
                  const degValue = Number.parseFloat(e.target.value || 0);
                  props.setThetaTool(degValue);
                }}
                step={2.5}
                min={jointLimits[6].min}
                max={jointLimits[6].max}
              />
            </div>
          </div>
        </div>
      </div>

      <div className="left-arm">
        <span>Left Arm</span>
        <div className="joint controller-controll-panel row">
          {theta_body_left.map((theta, idx) => (
            <div className="row mb-0" key={idx}>
              <div className="col-md-4">
                <label htmlFor={`theta_${idx+1}`} className="form-label">
                  <span className="form-control-plaintext">{`theta_${idx+1}`}</span>
                </label>
              </div>
              <div className="col-md-8">
                <input
                  type="number"
                  className="form-control"
                  id={`theta_left_${idx+1}`}
                  value={rad2deg(theta).toFixed(2)}
                  onChange={e => {
                    const degValue = Number.parseFloat(e.target.value || 0);
                    const newTheta = [...theta_body_left];
                    newTheta[idx] = deg2rad(degValue);
                    props.setThetaBodyLeft(newTheta);
                  }}
                  step={0.5}
                  min={jointLimits[idx].min}
                  max={jointLimits[idx].max}
                />
              </div>
            </div>
          ))}
          <div className="row mb-0">
            <div className="col-md-4">
              <label htmlFor="theta_tool" className="form-label">
                <span className="form-control-plaintext">theta_tool</span>
              </label>
            </div>
            <div className="col-md-8">
              <input
                type="number"
                className="form-control"
                id="theta_tool"
                value={theta_tool_left.toFixed(2)}
                onChange={e => {
                  const degValue = Number.parseFloat(e.target.value || 0);
                  props.setThetaToolLeft(degValue);
                }}
                step={2.5}
                min={jointLimits[6].min}
                max={jointLimits[6].max}
              />
            </div>
          </div>
        </div>
      </div>

      <div className="cam-arm">
        <span>Cam Arm</span>
        <div className="joint controller-controll-panel row">
          {theta_body_cam.map((theta, idx) => (
            <div className="row mb-0" key={idx}>
              <div className="col-md-4">
                <label htmlFor={`theta_${idx+1}`} className="form-label">
                  <span className="form-control-plaintext">{`theta_${idx+1}`}</span>
                </label>
              </div>
              <div className="col-md-8">
                <input
                  type="number"
                  className="form-control"
                  id={`theta_${idx+1}`}
                  value={rad2deg(theta).toFixed(2)}
                  onChange={e => {
                    const degValue = Number.parseFloat(e.target.value || 0);
                    const newTheta = [...theta_body_cam];
                    newTheta[idx] = deg2rad(degValue);
                    props.setThetaBodyCam(newTheta);
                  }}
                  step={0.5}
                  min={jointLimitsCam[idx].min}
                  max={jointLimitsCam[idx].max}
                />
              </div>
            </div>
          ))}
        </div>
      </div>              
    </>
    )
  }