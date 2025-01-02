import carb
import numpy as np
import omni

class KeyboardController:

    def __init__(self):
        self.command = None

    def read(self):
        """
        Subscribe to keyboard events
        """
        # subscribe to keyboard events
        app_window = omni.appwindow.get_default_app_window()  # noqa
        key_input = carb.input.acquire_input_interface()  # noqa
        key_input.subscribe_to_keyboard_events(app_window.get_keyboard(), self._sub_keyboard_event)
        return self.command

    def _sub_keyboard_event(self, event, *args, **kwargs):
        """subscribe to keyboard events, map to str"""
        # yapf: disable
        if (event.type == carb.input.KeyboardEventType.KEY_PRESS or
                event.type == carb.input.KeyboardEventType.KEY_REPEAT):
            if event.input == carb.input.KeyboardInput.W:
                self.command = 'w'
            if event.input == carb.input.KeyboardInput.S:
                self.command = 's'
            if event.input == carb.input.KeyboardInput.A:
                self.command = 'a'
            if event.input == carb.input.KeyboardInput.D:
                self.command = 'd'
            if event.input == carb.input.KeyboardInput.Q:
                self.command = 'q'
        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.command = None

