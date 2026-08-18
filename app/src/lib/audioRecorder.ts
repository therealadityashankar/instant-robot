// Audio recording controller using browser MediaRecorder to capture audio for Gemini multimodal input

export class AudioRecorder {
  private mediaRecorder: MediaRecorder | null = null;
  private stream: MediaStream | null = null;
  private chunks: BlobPart[] = [];
  private isRecording = false;

  async start(): Promise<void> {
    if (this.isRecording) {
      this.stop();
    }

    this.stream = await navigator.mediaDevices.getUserMedia({
      audio: {
        channelCount: 1,
        sampleRate: 16000,
        echoCancellation: true,
        noiseSuppression: true,
      },
    });

    this.chunks = [];
    const mimeType = MediaRecorder.isTypeSupported('audio/webm;codecs=opus')
      ? 'audio/webm;codecs=opus'
      : MediaRecorder.isTypeSupported('audio/webm')
        ? 'audio/webm'
        : 'audio/mp4';

    this.mediaRecorder = new MediaRecorder(this.stream, { mimeType });
    this.mediaRecorder.ondataavailable = (e) => {
      if (e.data && e.data.size > 0) {
        this.chunks.push(e.data);
      }
    };

    this.mediaRecorder.start(100);
    this.isRecording = true;
  }

  async stop(): Promise<{ blob: Blob; base64: string; mimeType: string; format: string }> {
    if (!this.mediaRecorder || !this.isRecording) {
      throw new Error('Not recording');
    }

    return new Promise((resolve, reject) => {
      if (!this.mediaRecorder) return reject(new Error('Recorder not initialized'));

      this.mediaRecorder.onstop = async () => {
        try {
          const mimeType = this.mediaRecorder?.mimeType || 'audio/webm';
          const blob = new Blob(this.chunks, { type: mimeType });

          // Clean up stream tracks
          if (this.stream) {
            for (const track of this.stream.getTracks()) track.stop();
            this.stream = null;
          }
          this.isRecording = false;

          const reader = new FileReader();
          reader.onloadend = () => {
            const dataUrl = reader.result as string;
            // Base64 without data: prefix
            const base64 = dataUrl.split(',')[1] || '';
            const format = mimeType.includes('webm') ? 'webm' : mimeType.includes('wav') ? 'wav' : 'mp3';
            resolve({ blob, base64, mimeType, format });
          };
          reader.onerror = reject;
          reader.readAsDataURL(blob);
        } catch (err) {
          reject(err);
        }
      };

      this.mediaRecorder.stop();
    });
  }

  cancel() {
    if (this.mediaRecorder && this.isRecording) {
      try {
        this.mediaRecorder.stop();
      } catch {}
    }
    if (this.stream) {
      for (const track of this.stream.getTracks()) track.stop();
      this.stream = null;
    }
    this.chunks = [];
    this.isRecording = false;
  }

  get active(): boolean {
    return this.isRecording;
  }
}

export const audioRecorder = new AudioRecorder();
