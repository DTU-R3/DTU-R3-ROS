#!/usr/bin/env python3
import aiy.assistant.grpc
import aiy.audio
import aiy.voicehat

import paho.mqtt.client as mqtt
import json

def send_cmd(str):
    topic = '/iot/commands/control/cc/alex-raspi-ros/voice'
    payload = json.dumps({"data": str})
    client = mqtt.Client("VOICEKIT")
    client.connect("localhost")
    client.publish(topic, payload)
    print('Send command '+ str)

def main():
    status_ui = aiy.voicehat.get_status_ui()
    status_ui.status('starting')
    assistant = aiy.assistant.grpc.get_assistant()
    button = aiy.voicehat.get_button()
    with aiy.audio.get_recorder():
        while True:
            status_ui.status('ready')
            print('Press the button and speak')
            button.wait_for_press()
            status_ui.status('listening')
            print('Listening...')
            text, audio = assistant.recognize()
            if text:
                if text == 'goodbye':
                    status_ui.status('stopping')
                    print('Bye!')
                    break
                print('You said "', text, '"')
            if audio:
                if text == "banana":
                    send_cmd('banana')
                    # aiy.audio.say('sent command banana')
                elif text == "start":
                    send_cmd('start')
                    aiy.audio.say('sent command start')
                elif text == "stop":
                    send_cmd('stop')
                    aiy.audio.say('sent command stop')
                elif text == "pause":
                    send_cmd('pause')
                    aiy.audio.say('sent command pause')
                else:
                    aiy.audio.play_audio(audio, assistant.get_volume())


if __name__ == '__main__':
    main()
