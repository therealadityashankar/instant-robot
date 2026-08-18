// Web Speech API wrapper for real-time speech-to-text recognition

export interface SpeechListenerOptions {
  onResult: (text: string, isFinal: boolean) => void;
  onError?: (err: string) => void;
  onStart?: () => void;
  onEnd?: () => void;
  lang?: string;
}

export function isSpeechSupported(): boolean {
  if (typeof window === 'undefined') return false;
  return 'webkitSpeechRecognition' in window || 'SpeechRecognition' in window;
}

export class SpeechController {
  private recognition: any = null;
  private isListening = false;
  private options: SpeechListenerOptions | null = null;

  constructor() {
    if (typeof window !== 'undefined') {
      const SpeechRecognition =
        (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition;
      if (SpeechRecognition) {
        this.recognition = new SpeechRecognition();
        this.recognition.continuous = false;
        this.recognition.interimResults = true;
        this.recognition.maxAlternatives = 1;
        this.recognition.lang = 'en-US';
      }
    }
  }

  start(options: SpeechListenerOptions): boolean {
    if (!this.recognition) {
      options.onError?.('Speech recognition is not supported in this browser.');
      return false;
    }

    if (this.isListening) {
      this.stop();
    }

    this.options = options;
    if (options.lang) {
      this.recognition.lang = options.lang;
    }

    this.recognition.onstart = () => {
      this.isListening = true;
      this.options?.onStart?.();
    };

    this.recognition.onresult = (event: any) => {
      let interim = '';
      let final = '';

      for (let i = event.resultIndex; i < event.results.length; ++i) {
        const transcript = event.results[i][0].transcript;
        if (event.results[i].isFinal) {
          final += transcript;
        } else {
          interim += transcript;
        }
      }

      if (final) {
        this.options?.onResult(final.trim(), true);
      } else if (interim) {
        this.options?.onResult(interim.trim(), false);
      }
    };

    this.recognition.onerror = (event: any) => {
      this.isListening = false;
      const errorMsg = event.error === 'not-allowed'
        ? 'Microphone access denied.'
        : `Speech error: ${event.error}`;
      this.options?.onError?.(errorMsg);
    };

    this.recognition.onend = () => {
      this.isListening = false;
      this.options?.onEnd?.();
    };

    try {
      this.recognition.start();
      return true;
    } catch (e: any) {
      options.onError?.(e.message || 'Failed to start speech recognition');
      return false;
    }
  }

  stop() {
    if (this.recognition && this.isListening) {
      try {
        this.recognition.stop();
      } catch {}
      this.isListening = false;
    }
  }

  get active(): boolean {
    return this.isListening;
  }
}

export const speechController = new SpeechController();
