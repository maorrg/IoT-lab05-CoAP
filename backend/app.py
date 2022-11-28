from aiocoap import *
from flask import Flask, session
import asyncio
import json
from mixpanel import Mixpanel
mp = Mixpanel("MIXPANEL_API_TOKEN")

def observe_callback(response):
    if response.code.is_successful():
        data_json = json.loads(response.payload)
        humidity = data_json['humidity']
        temperature = data_json['temperature']
        print(humidity, temperature)
        mp.track('humidity', humidity)
        mp.track('temperature', temperature)

async def observer_server():
    global session
    context = await Context.create_client_context()
    request = Message(code=GET)
    request.set_request_uri('coap://10.0.2.72/alarm')
    request.opt.observe = 0
    observation_is_over = asyncio.Future()
    try:
        context_request = context.request(request)
        context_request.observation.register_callback(observe_callback)
        _ = await context_request.response
        exit_reason = await observation_is_over
        print('Observation is over: %r' % exit_reason)
    finally:
        if not context_request.response.done():
            context_request.response.cancel()
        if not context_request.observation.cancelled:
            context_request.observation.cancel()

def observer():
    asyncio.run(observer_server())
            
if __name__ == '__main__':
    observer()