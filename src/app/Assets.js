import React from 'react';

const robotAssets = {
  agilex_piper: [
    { id: "base", src: "/agilex_piper/base_link_lite.glb" },
    { id: "j1", src: "/agilex_piper/link1_lite.glb" },
    { id: "j2", src: "/agilex_piper/link2_lite.glb" },
    { id: "j3", src: "/agilex_piper/link3_lite.glb" },
    { id: "j4", src: "/agilex_piper/link4_lite.glb" },
    { id: "j5", src: "/agilex_piper/link5_lite.glb" },
    { id: "j6", src: "/agilex_piper/link6_lite.glb" },
    { id: "j6_1", src: "/agilex_piper/link7_lite.glb" },
    { id: "j6_2", src: "/agilex_piper/link8_lite.glb" },
  ],
  jaka_zu_5: [
    { id: "base", src: "/jaka_zu_5/JAKA_Zu_5_BASE.gltf" },
    { id: "j1", src: "/jaka_zu_5/JAKA_Zu_5_J1.gltf" },
    { id: "j2", src: "/jaka_zu_5/JAKA_Zu_5_J2.gltf" },
    { id: "j3", src: "/jaka_zu_5/JAKA_Zu_5_J3.gltf" },
    { id: "j4", src: "/jaka_zu_5/JAKA_Zu_5_J4.gltf" },
    { id: "j5", src: "/jaka_zu_5/JAKA_Zu_5_J5.gltf" },
    { id: "j6", src: "/jaka_zu_5/JAKA_Zu_5_J6.gltf" },
  ],
  myCobot280: [
    { id: "base", src: "/myCobot280/link0.glb" },
    { id: "j1", src: "/myCobot280/link1.glb" },
    { id: "j2", src: "/myCobot280/link2.glb" },
    { id: "j3", src: "/myCobot280/link3.glb" },
    { id: "j4", src: "/myCobot280/link4.glb" },
    { id: "j5", src: "/myCobot280/link5.glb" },
    { id: "j6", src: "/myCobot280/link6.glb" },
    { id: "cam_mount_1", src: "/myCobot280/zedm_mount_1.glb" },
    { id: "cam_mount_2", src: "/myCobot280/zedm_mount_2.glb" },
    { id: "cam", src: "/myCobot280/zedm.glb" },
  ],
};

const Assets = ({ robot_list }) => {
  if (!robot_list || robot_list.length === 0) {
    console.warn("No robots provided in robot_list.");
    return null;
  }

  return (
    <a-assets>
      {robot_list.map(({ robotId, robot_model }) => {
        if (!robot_model) {
          console.warn(`No robot_model defined for robotId: ${robotId}`);
          return null;
        }

        const assets = robotAssets[robot_model];
        if (!assets) {
          console.warn(`No assets found for robot model: ${robot_model}`);
          return null;
        }

        return assets.map(({ id, src }) => (
          <a-asset-items key={`${robotId}_${id}`} id={`${robotId}_${id}`} src={src}></a-asset-items>
        ));
      })}
    </a-assets>
  );
};

export default Assets;