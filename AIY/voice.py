#!/usr/bin/env python3
import logging
import platform
import sys
import threading

from google.assistant.library.event import EventType

from aiy.assistant import auth_helpers
from aiy.assistant.library import Assistant
from aiy.board import Board, Led

import paho.mqtt.client as mqtt
import json

class MyAssistant:
    def __init__(self):
        self._task = threading.Thread(target=self._run_task)
        self._can_start_conversation = False
        self._assistant = None
        self._board = Board()
        self._board.button.when_pressed = self._on_button_pressed

    def start(self):
        self._task.start()

    def _run_task(self):
        credentials = auth_helpers.get_assistant_credentials()
        with Assistant(credentials) as assistant:
            self._assistant = assistant
            for event in assistant.start():
                self._process_event(event)

    def _process_event(self, event):
        logging.info(event)
        if event.type == EventType.ON_START_FINISHED:
            self._board.led.status = Led.BEACON_DARK  # Ready.
            self._can_start_conversation = True
            # Start the voicehat button trigger.
            logging.info('Press the button, then speak. '
                         'Press Ctrl+C to quit...')

        elif event.type == EventType.ON_CONVERSATION_TURN_STARTED:
            self._can_start_conversation = False
            self._board.led.state = Led.ON  # Listening.

        elif event.type == EventType.ON_END_OF_UTTERANCE:
            self._board.led.state = Led.PULSE_QUICK  # Thinking.

        elif event.type == EventType.ON_RECOGNIZING_SPEECH_FINISHED and event.args:
            print('You said:', event.args['text'])
            text = event.args['text'].lower()
            if text == "banana":
                self._assistant.stop_conversation()
                self._send_cmd('banana')
            elif text == "apple":
                self._assistant.stop_conversation()
                self._send_cmd('apple')
            elif text == "orange":
                self._assistant.stop_conversation()
                self._send_cmd('orange')
            elif text == "start":
                self._assistant.stop_conversation()
                self._send_cmd('start')
                aiy.voice.tts.say('sent command start')
            elif text == "stop":
                self._assistant.stop_conversation()
                self._send_cmd('stop')
                aiy.voice.tts.say('sent command stop')
            elif text == "pause":
                self._assistant.stop_conversation()
                self._send_cmd('pause')
                aiy.voice.tts.say('sent command pause')

        elif (event.type == EventType.ON_CONVERSATION_TURN_FINISHED
              or event.type == EventType.ON_CONVERSATION_TURN_TIMEOUT
              or event.type == EventType.ON_NO_RESPONSE):
            self._board.led.state = Led.BEACON_DARK  # Ready.
            self._can_start_conversation = True

        elif event.type == EventType.ON_ASSISTANT_ERROR and event.args and event.args['is_fatal']:
            sys.exit(1)

    def _on_button_pressed(self):
        # Check if we can start a conversation. 'self._can_start_conversation'
        # is False when either:
        # 1. The assistant library is not yet ready; OR
        # 2. The assistant library is already in a conversation.
        if self._can_start_conversation:
            self._assistant.start_conversation()

    def _send_cmd(self, str):
        topic = '/iot/commands/control/cc/alex-raspi-ros/voice'
        payload = json.dumps({"data": str})
        client = mqtt.Client("VOICEKIT")
        client.connect("localhost")
        client.publish(topic, payload)
        print('Send command '+ str)

def main():
    logging.basicConfig(level=logging.INFO)
    MyAssistant().start()

if __name__ == '__main__':
    main()
