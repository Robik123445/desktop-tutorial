import React, { useState } from 'react';
import log from './logger';

/**
 * Simple feedback form that logs user messages.
 */
export default function FeedbackForm() {
  const [text, setText] = useState('');

  const send = () => {
    const msg = text.trim();
    if (!msg) return;
    log(`feedback: ${msg}`);
    alert('Thanks for the feedback!');
    setText('');
  };

  return (
    <div className="p-2 space-y-2">
      <textarea
        value={text}
        onChange={(e) => setText(e.target.value)}
        className="w-full p-1 text-black rounded"
        placeholder="Your feedback"
        data-testid="feedback-input"
      />
      <button
        onClick={send}
        className="bg-blue-600 text-white px-3 py-1 rounded"
        data-testid="feedback-send"
      >
        Send Feedback
      </button>
    </div>
  );
}
