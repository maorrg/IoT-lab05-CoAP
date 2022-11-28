import threading
import asyncio
import aiocoap.resource as resource
import aiocoap

class AlarmResource(resource.ObservableResource):
    """This resource supports the GET and PUT methods and is observable.
    GET: Return current state of alarm
    PUT: Update state of alarm and notify registered observers
    """

    def __init__(self):
        super().__init__()

        self.status = "OFF"
        self.has_observers = False
        self.notify_observers = False

    # Ensure observers are notify if required
    def notify_observers_check(self):
        while True:
            if self.has_observers and self.notify_observers:
                print('Notifying observers')
                self.updated_state()
                self.notify_observers = False

    # Observers change event callback
    def update_observation_count(self, count):
        if count:
            self.has_observers = True
        else:
            self.has_observers = False

    # Handles GET request or observer notify
    async def render_get(self, request):
        print('Return alarm state: %s' % self.status)
        payload = b'%s' % self.status.encode('ascii')

        return aiocoap.Message(payload=payload)

    # Handles PUT request
    async def render_put(self, request):
        self.status = request.payload.decode('ascii')
        print('Update alarm state: %s' % self.status)
        self.notify_observers = True

        return aiocoap.Message(code=aiocoap.CHANGED, payload=b'%s' % self.status.encode('ascii'))
    
def main():
    # Resource tree creation
    root = resource.Site()
    alarm_resource = AlarmResource()
    root.add_resource(['alarm'], alarm_resource)
    asyncio.Task(aiocoap.Context.create_server_context(root, bind=('10.0.2.72', 5683)))

    # Spawn a daemon to notify observers when alarm status changes
    observers_notifier = threading.Thread(target=alarm_resource.notify_observers_check)
    observers_notifier.daemon = True
    observers_notifier.start()

    asyncio.get_event_loop().run_forever()

if __name__ == "__main__":
    main()