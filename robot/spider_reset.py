"""
  Reset spider driver during start new app
"""

from osgar.node import Node


class Reset(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('reset')

    def run(self):
        self.publish('reset', True)
