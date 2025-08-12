import React, { useRef, useState, useEffect } from 'react';
import log from './src/log.js';

/**
 * Webcam feed with mock object detection. Shows warning when a person or hand is detected.
 */
export default function WebcamDetector({ testDetection = null }) {
  const videoRef = useRef(null);
  const streamRef = useRef(null);
  const [active, setActive] = useState(false);
  const [warning, setWarning] = useState(false);

  // start webcam and mock detection
  const start = async () => {
    if (active) return;
    log('Start detection clicked');
    try {
      // open webcam
      streamRef.current = await navigator.mediaDevices.getUserMedia({ video: true });
      videoRef.current.srcObject = streamRef.current;
      await videoRef.current.play();
      log('Webcam started');
      setActive(true);
      // mock detection delay
      setTimeout(() => {
        const detected = testDetection || 'person';
        if (detected === 'person' || detected === 'hand') {
          log(`Warning: ${detected} detected`);
          setWarning(true);
        }
      }, 1000);
    } catch (err) {
      console.error('Failed to access webcam:', err);
    }
  };

  // cleanup stream on unmount
  useEffect(() => {
    return () => {
      if (streamRef.current) {
        streamRef.current.getTracks().forEach((t) => t.stop());
      }
    };
  }, []);

  return (
    <div className="p-4 space-y-2 max-w-md mx-auto">
      <video ref={videoRef} className="w-full h-48 bg-black" data-testid="video" />
      <button
        onClick={start}
        disabled={active}
        className="bg-blue-600 text-white px-3 py-1 rounded hover:bg-blue-700 disabled:opacity-50"
      >
        Start Object Detection
      </button>
      {warning && (
        <div
          className="bg-red-600 text-white p-2 mt-2 font-bold"
          data-testid="warning"
        >
          HAND DETECTED! EMERGENCY STOP!
        </div>
      )}
    </div>
  );
}
