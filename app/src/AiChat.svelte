<script lang="ts">
  import { settings } from './lib/settings.svelte';
  import { audioRecorder } from './lib/audioRecorder';
  import {
    sendOpenRouterChatMessage,
    type ChatMessage,
    type RobotStateContext,
    type RobotActionCallbacks,
  } from './lib/aiAssistant';

  let {
    context,
    callbacks,
  }: {
    context: RobotStateContext;
    callbacks: RobotActionCallbacks;
  } = $props();

  let messages = $state<ChatMessage[]>([]);
  let inputText = $state('');
  let busy = $state(false);
  let isRecording = $state(false);
  let errorMsg = $state<string | null>(null);
  let missingKeyPrompt = $state(false);
  let chatContainer = $state<HTMLDivElement>();

  function scrollToBottom() {
    setTimeout(() => {
      if (chatContainer) chatContainer.scrollTop = chatContainer.scrollHeight;
    }, 50);
  }

  async function handleSendText(textToSend?: string) {
    const text = (textToSend || inputText).trim();
    if (!text || busy) return;

    if (!settings.openrouterApiKey) {
      missingKeyPrompt = true;
      errorMsg = null;
      return;
    }

    missingKeyPrompt = false;
    errorMsg = null;
    inputText = '';

    const userMsg: ChatMessage = {
      id: `msg-${Date.now()}`,
      role: 'user',
      content: text,
      timestamp: Date.now(),
    };

    messages = [...messages, userMsg];
    scrollToBottom();
    busy = true;

    try {
      const response = await sendOpenRouterChatMessage(
        settings.openrouterApiKey,
        settings.openrouterModel,
        messages,
        context,
        callbacks,
      );
      messages = [...messages, response];
      scrollToBottom();
    } catch (err: any) {
      errorMsg = err.message || 'Failed to send message.';
    } finally {
      busy = false;
    }
  }

  async function toggleRecording() {
    if (busy) return;

    if (!settings.openrouterApiKey) {
      missingKeyPrompt = true;
      errorMsg = null;
      return;
    }

    missingKeyPrompt = false;
    errorMsg = null;

    if (isRecording) {
      // Stop recording and send audio directly to Gemini
      isRecording = false;
      busy = true;
      try {
        const { base64, mimeType, format } = await audioRecorder.stop();

        const userVoiceMsg: ChatMessage = {
          id: `msg-voice-${Date.now()}`,
          role: 'user',
          content: '🎙️ Voice command',
          timestamp: Date.now(),
          audioBase64: base64,
          audioMimeType: mimeType,
          audioFormat: format,
        };

        messages = [...messages, userVoiceMsg];
        scrollToBottom();

        const response = await sendOpenRouterChatMessage(
          settings.openrouterApiKey,
          settings.openrouterModel,
          messages,
          context,
          callbacks,
        );
        messages = [...messages, response];
        scrollToBottom();
      } catch (err: any) {
        errorMsg = err.message || 'Failed to process voice recording.';
      } finally {
        busy = false;
      }
    } else {
      // Start recording
      try {
        await audioRecorder.start();
        isRecording = true;
      } catch (err: any) {
        errorMsg = err.message || 'Failed to access microphone.';
      }
    }
  }

  function handleKeyDown(e: KeyboardEvent) {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendText();
    }
  }
</script>

<div class="aichat">
  {#if messages.length > 0}
    <div class="messages" bind:this={chatContainer}>
      {#each messages as msg (msg.id)}
        <div class="msg {msg.role}">
          <div class="bubble">
            {#if msg.audioBase64}
              <span class="voicebadge">🎙️ Audio message</span>
            {/if}
            <div class="content">{msg.content}</div>
            {#if msg.toolCalls}
              <div class="toolcalls">
                {#each msg.toolCalls as tc}
                  <span class="toolbadge">⚡ {tc.result || tc.name}</span>
                {/each}
              </div>
            {/if}
          </div>
        </div>
      {/each}
      {#if busy}
        <div class="msg assistant">
          <div class="bubble thinking">
            <span class="dot"></span><span class="dot"></span><span class="dot"></span>
          </div>
        </div>
      {/if}
    </div>
  {/if}

  <!-- Minimalist, seamless flush input group: Voice Button + Text Input + Send Button -->
  <div class="inputgroup" class:has-focus={false}>
    <button
      type="button"
      class="segment-btn micbtn"
      class:recording={isRecording}
      disabled={busy}
      onclick={toggleRecording}
      title={isRecording ? 'Click to finish speaking & send to Gemini' : 'Speak command to Gemini'}
      aria-label="Voice input"
    >
      <svg
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
        stroke-linejoin="round"
      >
        <path d="M12 2a3 3 0 0 0-3 3v7a3 3 0 0 0 6 0V5a3 3 0 0 0-3-3Z" />
        <path d="M19 10v2a7 7 0 0 1-14 0v-2" />
        <line x1="12" x2="12" y1="19" y2="22" />
      </svg>
      {#if isRecording}<span class="recpulse"></span>{/if}
    </button>

    <input
      type="text"
      placeholder={isRecording ? 'Listening to voice…' : 'Ask Gemini or speak command…'}
      bind:value={inputText}
      disabled={busy || isRecording}
      onkeydown={handleKeyDown}
      aria-label="Command text"
    />

    <button
      type="button"
      class="segment-btn sendbtn"
      class:active={!!inputText.trim()}
      disabled={!inputText.trim() || busy || isRecording}
      onclick={() => handleSendText()}
      aria-label="Send command"
      title="Send command"
    >
      <svg
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
        stroke-linejoin="round"
      >
        <line x1="22" y1="2" x2="11" y2="13" />
        <polygon points="22 2 15 22 11 13 2 9 22 2" />
      </svg>
    </button>
  </div>

  {#if missingKeyPrompt}
    <div class="keyprompt-banner">
      <span>Please set an OpenRouter API key to use Gemini AI.</span>
      <button
        class="open-settings-link"
        onclick={() => {
          settings.open = true;
          missingKeyPrompt = false;
        }}
      >
        Open Settings →
      </button>
    </div>
  {/if}

  {#if errorMsg}
    <div class="errbar">{errorMsg}</div>
  {/if}
</div>

<style>
  .aichat {
    background: transparent;
    border: none;
    padding: 0.25rem 0;
    margin-top: 0.4rem;
    display: flex;
    flex-direction: column;
    gap: 0.4rem;
    font-family: inherit;
  }

  .keyprompt-banner {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.4rem;
    background: #eff6ff;
    border: 1px solid #bfdbfe;
    border-radius: 6px;
    padding: 0.4rem 0.6rem;
    font-size: 0.75rem;
    color: #1e40af;
    animation: fadeIn 0.2s ease-in-out;
  }

  .open-settings-link {
    background: #2563eb;
    color: #ffffff;
    border: none;
    border-radius: 4px;
    padding: 0.2rem 0.5rem;
    font-size: 0.72rem;
    font-weight: 500;
    cursor: pointer;
    white-space: nowrap;
    transition: background 0.15s;
  }

  .open-settings-link:hover {
    background: #1d4ed8;
  }

  @keyframes fadeIn {
    from { opacity: 0; transform: translateY(-2px); }
    to { opacity: 1; transform: translateY(0); }
  }

  .messages {
    max-height: 180px;
    overflow-y: auto;
    display: flex;
    flex-direction: column;
    gap: 0.4rem;
    padding: 0.4rem;
    background: var(--surface-2, #f9fafb);
    border-radius: 6px;
    border: 1px solid var(--line-soft, #e5e7eb);
  }

  .msg {
    display: flex;
    width: 100%;
  }

  .msg.user {
    justify-content: flex-end;
  }

  .msg.assistant {
    justify-content: flex-start;
  }

  .bubble {
    max-width: 88%;
    padding: 0.35rem 0.55rem;
    border-radius: 6px;
    font-size: 0.78rem;
    line-height: 1.35;
  }

  .msg.user .bubble {
    background: #2563eb;
    color: #ffffff;
  }

  .msg.assistant .bubble {
    background: var(--surface, #ffffff);
    color: var(--ink, #111827);
    border: 1px solid var(--line-soft, #e5e7eb);
  }

  .voicebadge {
    display: block;
    font-size: 0.65rem;
    opacity: 0.85;
    margin-bottom: 2px;
  }

  .toolcalls {
    display: flex;
    flex-direction: column;
    gap: 2px;
    margin-top: 4px;
  }

  .toolbadge {
    font-size: 0.65rem;
    background: rgba(34, 197, 94, 0.15);
    color: #15803d;
    padding: 2px 4px;
    border-radius: 4px;
    display: inline-block;
  }

  .thinking {
    display: flex;
    align-items: center;
    gap: 4px;
    padding: 0.4rem 0.8rem;
  }

  .dot {
    width: 5px;
    height: 5px;
    border-radius: 50%;
    background: #94a3b8;
    animation: bounce 1.2s infinite ease-in-out;
  }
  .dot:nth-child(2) { animation-delay: 0.2s; }
  .dot:nth-child(3) { animation-delay: 0.4s; }

  @keyframes bounce {
    0%, 80%, 100% { transform: translateY(0); }
    40% { transform: translateY(-4px); }
  }

  .errbar {
    font-size: 0.72rem;
    color: #dc2626;
    background: #fef2f2;
    padding: 0.3rem 0.5rem;
    border-radius: 4px;
    border: 1px solid #fca5a5;
  }

  /* Seamless, flush input group */
  .inputgroup {
    display: flex;
    align-items: stretch;
    border: 1px solid var(--line-soft, #d1d5db);
    border-radius: 6px;
    background: var(--surface, #ffffff);
    overflow: hidden;
    box-shadow: 0 1px 2px rgba(0, 0, 0, 0.03);
    transition: border-color 0.15s ease, box-shadow 0.15s ease;
  }

  .inputgroup:focus-within {
    border-color: #3b82f6;
    box-shadow: 0 0 0 1px #3b82f6;
  }

  .inputgroup input {
    flex: 1;
    min-width: 0;
    border: none;
    background: transparent;
    padding: 0.42rem 0.55rem;
    font-size: 0.78rem;
    color: var(--ink, #111827);
    outline: none;
  }

  .segment-btn {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    min-height: auto;
    width: 32px;
    padding: 0;
    border: none;
    border-radius: 0;
    background: var(--surface-2, #f9fafb);
    color: var(--muted, #6b7280);
    box-shadow: none;
    cursor: pointer;
    transition: background-color 0.15s ease, color 0.15s ease;
    flex-shrink: 0;
  }

  .segment-btn:hover:not(:disabled) {
    background: #f3f4f6;
    color: var(--ink, #111827);
  }

  .segment-btn:disabled {
    opacity: 0.4;
    cursor: not-allowed;
  }

  .micbtn {
    border-right: 1px solid var(--line-soft, #e5e7eb);
    position: relative;
  }

  .micbtn svg {
    width: 15px;
    height: 15px;
  }

  .micbtn.recording {
    background: #dc2626;
    color: #ffffff;
  }

  .recpulse {
    position: absolute;
    inset: -2px;
    border-radius: 4px;
    border: 2px solid #dc2626;
    animation: pulse 1.5s infinite;
  }

  @keyframes pulse {
    0% { transform: scale(1); opacity: 0.8; }
    100% { transform: scale(1.2); opacity: 0; }
  }

  .sendbtn {
    border-left: 1px solid var(--line-soft, #e5e7eb);
  }

  .sendbtn svg {
    width: 14px;
    height: 14px;
  }

  .sendbtn.active {
    color: #2563eb;
    background: rgba(37, 99, 235, 0.08);
  }

  .sendbtn.active:hover {
    background: #2563eb;
    color: #ffffff;
  }
</style>
