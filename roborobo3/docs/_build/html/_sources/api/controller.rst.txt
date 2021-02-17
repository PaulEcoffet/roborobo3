.. currentmodule:: pyroborobo

Controller
==========

Documentation
-------------

.. autoclass:: Controller
    :members:

Template
--------

If you want to create a subclass of a controller, you can use this template:

.. code-block:: python

    class MyController(Controller):
        def __init__(self, wm):
            super().__init__(wm)
            self.rob = Pyroborobo.get()

        def reset(self):
            super().reset()

        def step(self):
            super().step()
