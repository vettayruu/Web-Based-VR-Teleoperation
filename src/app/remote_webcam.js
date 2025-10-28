import * as React from 'react';
import Script from 'next/script';

export default function RemoteWebcam({ onVideoStream1, onVideoStream2, onVideoStream3 }) {
  const [soraReady, setSoraReady] = React.useState(false);

  React.useEffect(() => {
    if (soraReady && window.Sora) {
      // Video streaming setup
      // const signalingUrl = 'wss://sora.uclab.jp/signaling';
      const signalingUrl = 'wss://sora2.uclab.jp/signaling';
      const sora = window.Sora.connection(signalingUrl);

      const options = {
        role: 'recvonly',
        multistream: false,
        video: { codecType: 'VP8', resolution: '720p', bitrate: 5000 },
        audio: false,
      };

      // Webcam 1
      const recvonly_webcam1 = sora.recvonly('sora_liust_left', options);
      recvonly_webcam1.on('track', event => {
        if (event.track.kind === 'video') {
          const mediaStream = new MediaStream();
          mediaStream.addTrack(event.track);
          if (onVideoStream1) onVideoStream1(mediaStream);
        }
      });
      recvonly_webcam1.connect();

      // Webcam 2
      const recvonly_webcam2 = sora.recvonly('sora_liust_right', options);
      recvonly_webcam2.on('track', event => {
        if (event.track.kind === 'video') {
          const mediaStream = new MediaStream();
          mediaStream.addTrack(event.track);
          if (onVideoStream2) onVideoStream2(mediaStream);
        }
      });
      recvonly_webcam2.connect();

      // Webcam 3
      const recvonly_webcam3 = sora.recvonly('sora_liust_sub', options);
      recvonly_webcam3.on('track', event => {
        if (event.track.kind === 'video') {
          const mediaStream = new MediaStream();
          mediaStream.addTrack(event.track);
          if (onVideoStream3) onVideoStream3(mediaStream);
        }
      });
      recvonly_webcam3.connect();

    }
  }, [soraReady, onVideoStream1, onVideoStream2, onVideoStream3]);

  return (
    <>
      <Script
        src="https://cdn.jsdelivr.net/npm/sora-js-sdk@2021.1.1/dist/sora.min.js"
        strategy="lazyOnload"
        onLoad={() => setSoraReady(true)}
      />
    </>
  );
}