# Voice AI with OpenAI Whisper

## Introduction

**OpenAI Whisper** is a state-of-the-art speech recognition system that enables robots to understand natural language voice commands.

## How Whisper Works

Whisper is a **transformer-based** model trained on 680,000 hours of multilingual speech data. It achieves:

- **95%+ accuracy** on clean speech
- **Robust performance** in noisy environments
- **99 languages** supported
- **Under 2 seconds** transcription latency

## Architecture Overview

```
┌──────────────┐
│  Microphone  │
└──────┬───────┘
       │ Audio Stream
       ▼
┌──────────────────┐
│  MediaRecorder   │
│  (Browser API)   │
└──────┬───────────┘
       │ WebM File
       ▼
┌──────────────────┐
│  FastAPI Backend │
│  /voice/transcribe│
└──────┬───────────┘
       │ HTTP POST
       ▼
┌──────────────────┐
│  Whisper API     │
│  (OpenAI Cloud)  │
└──────┬───────────┘
       │ Transcription
       ▼
┌──────────────────┐
│  Robot Command   │
│  Processing      │
└──────────────────┘
```

## Implementation

### Frontend: Recording Audio

```typescript
// useVoiceRecorder.ts
import { useState, useRef } from 'react';

export function useVoiceRecorder() {
  const [isRecording, setIsRecording] = useState(false);
  const [transcription, setTranscription] = useState('');
  const mediaRecorder = useRef<MediaRecorder | null>(null);

  const startRecording = async () => {
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    const recorder = new MediaRecorder(stream, { mimeType: 'audio/webm' });

    const audioChunks: Blob[] = [];
    recorder.ondataavailable = (event) => audioChunks.push(event.data);

    recorder.onstop = async () => {
      const audioBlob = new Blob(audioChunks, { type: 'audio/webm' });
      await transcribeAudio(audioBlob);
    };

    recorder.start();
    mediaRecorder.current = recorder;
    setIsRecording(true);
  };

  const stopRecording = () => {
    mediaRecorder.current?.stop();
    setIsRecording(false);
  };

  const transcribeAudio = async (audioBlob: Blob) => {
    const formData = new FormData();
    formData.append('audio_file', audioBlob, 'command.webm');

    const response = await fetch('/api/v1/voice/transcribe', {
      method: 'POST',
      body: formData,
    });

    const data = await response.json();
    setTranscription(data.transcribed_text);
  };

  return { isRecording, transcription, startRecording, stopRecording };
}
```

### Backend: Whisper API Integration

```python
# backend/src/api/routes/voice.py
from fastapi import APIRouter, UploadFile, HTTPException
import openai
import os

router = APIRouter()

@router.post("/voice/transcribe")
async def transcribe_voice(audio_file: UploadFile):
    """
    Transcribe voice command using OpenAI Whisper API
    """
    # Validate audio file
    if audio_file.size > 5_000_000:  # 5MB limit
        raise HTTPException(400, "Audio file too large (max 5MB)")

    # Read audio bytes
    audio_bytes = await audio_file.read()

    # Call Whisper API
    try:
        response = openai.Audio.transcribe(
            model="whisper-1",
            file=audio_bytes,
            response_format="json"
        )

        return {
            "transcribed_text": response["text"],
            "confidence_score": 0.95,  # Whisper doesn't return confidence
            "language_detected": response.get("language", "en")
        }

    except Exception as e:
        raise HTTPException(500, f"Transcription failed: {str(e)}")
```

### UI Component

```tsx
// VoiceCommandUI.tsx
import React from 'react';
import { motion } from 'framer-motion';
import { useVoiceRecorder } from '../hooks/useVoiceRecorder';

export function VoiceCommandUI() {
  const { isRecording, transcription, startRecording, stopRecording } = useVoiceRecorder();

  return (
    <div className="voice-command-panel">
      <h3>Voice Control</h3>

      {/* Microphone Button */}
      <motion.button
        onClick={isRecording ? stopRecording : startRecording}
        animate={isRecording ? { scale: [1, 1.1, 1] } : {}}
        transition={{ repeat: Infinity, duration: 1 }}
        className={isRecording ? 'recording' : ''}
      >
        {isRecording ? '🔴 Recording...' : '🎤 Press to Speak'}
      </motion.button>

      {/* Transcription Display */}
      {transcription && (
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          className="transcription-result"
        >
          <p><strong>You said:</strong> "{transcription}"</p>
          <button onClick={() => sendCommand(transcription)}>
            Execute Command
          </button>
        </motion.div>
      )}
    </div>
  );
}
```

## Best Practices

### 1. Audio Quality

```typescript
// Request high-quality audio
const constraints = {
  audio: {
    echoCancellation: true,
    noiseSuppression: true,
    sampleRate: 16000,  // Whisper optimal sample rate
  }
};
```

### 2. Error Handling

```python
# Graceful degradation
try:
    transcription = whisper_client.transcribe(audio)
except RateLimitError:
    return {"error": "API rate limit reached. Please try again in 1 minute."}
except NetworkError:
    return {"error": "Connection failed. Check your internet."}
```

### 3. Caching

```python
import hashlib
import redis

def transcribe_with_cache(audio_bytes: bytes) -> str:
    # Cache by audio content hash
    audio_hash = hashlib.md5(audio_bytes).hexdigest()

    cached = redis_client.get(f"transcription:{audio_hash}")
    if cached:
        return cached.decode()

    transcription = openai.Audio.transcribe("whisper-1", audio_bytes)
    redis_client.setex(f"transcription:{audio_hash}", 300, transcription)  # 5min TTL

    return transcription
```

## Common Issues

### Issue: "Browser doesn't support audio recording"

**Solution**: Check for MediaRecorder API support

```typescript
if (!navigator.mediaDevices || !window.MediaRecorder) {
  alert('Your browser does not support audio recording. Please use Chrome or Firefox.');
}
```

### Issue: High latency (>5 seconds)

**Solution**: Optimize audio file size

```typescript
// Limit recording duration
setTimeout(() => {
  if (isRecording) {
    stopRecording();
    alert('Maximum recording length is 60 seconds');
  }
}, 60000);
```

### Issue: Poor transcription accuracy

**Solution**: Improve audio quality and add context

```python
# Provide context to Whisper
response = openai.Audio.transcribe(
    model="whisper-1",
    file=audio_bytes,
    language="en",  # Specify language for better accuracy
    prompt="Robot commands like move forward, turn left, pick up object"
)
```

## Performance Metrics

Target performance for production:

- **Latency**: Under 2 seconds (audio upload + transcription)
- **Accuracy**: Over 90% word error rate (WER)
- **Availability**: 99.9% uptime (use retry logic)
- **Cost**: ~$0.006 per minute of audio

## Next Steps

Now that you can transcribe voice commands, the next step is to **convert natural language into robot actions** using GPT-4 task planning!

Continue to: [LLM Task Planning →](../llm-planning/gpt4-integration.md)
