import system.settings as settings
import websocket
import threading
import json

class Client:
    def __init__(self, loopfunction, presenter):
        if settings.NEURAL_DATA_TYPE != "frequency":
            raise SyntaxError("Invalid datatype, server only supports the frequency datatype")
        self.loopfunction = loopfunction
        self.presenter = presenter
        self.frequencies = [] * settings.NEURAL_ELECTRODES_TOTAL
        websocket.enableTrace(True)
        interval = int(1000/settings.LED_REFRESHES_PER_SECOND)
        request = "ws://" + settings.SERVER_IP + ":" + settings.SERVER_PORT + "/data/" + str(interval)
        self.ws = websocket.WebSocketApp(request,
                                         on_message=self._on_message,
                                         on_error=self._on_error,
                                         on_close=self._on_close,
                                         on_open=self._on_open)
        self.timer = threading.Timer(settings.SERVER_TIMEOUT + 1/settings.LED_REFRESHES_PER_SECOND,
                                     self._timer_out)

    def loop(self):
        self.ws.run_forever()
        print("test")

    def _timer_out(self):
        red = bytearray([255, 0, 0] * settings.LEDS_TOTAL)
        print("Slow connection, nothing received from server")
        self.presenter.refresh(red)

    def _on_close(self, ws):
        print("Connection closed")

    def _on_error(self, ws ,error):
        print("Connection error: {}".format(error))

    def _on_open(self, ws):
        print("Connection established")

    def _on_message(self, ws, message):
        self.timer.cancel()
        self.frequencies = json.loads(message)
        self.loopfunction(self.frequencies)
        self.timer = threading.Timer(settings.SERVER_TIMEOUT + 1 / settings.LED_REFRESHES_PER_SECOND,
                                     self._timer_out)
