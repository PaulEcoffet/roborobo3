# Object Creation

```eval_rst 
.. note::
    The source code for this tutorial can be found in py_example/objects.py and py_example/py_switch_example.py
```

It is possible to create custom objects with Pyroborobo. First of all, you must create a class that inherit from either `CircleObject` or `SquareObject`. Let's recreate the Gate and Switch object of the wanderer environment using python. An object must implement the reset and step function. The constructor receives a `data` dictionary containing all the information put in the configuration file. 

```python
from pyroborobo import SquareObject, CircleObject, Pyroborobo

class SwitchObject(CircleObject):
    def __init__(self, id, data):
        CircleObject.__init__(self, id)  # Do not forget to call super constructor
        self.regrow_time = data['regrowTimeMax']
        self.cur_regrow = 0
        self.triggered = False
        self.gate_id = data['sendMessageTo']
        self.rob = Pyroborobo.get() # Get pyroborobo singleton

    def reset(self):
        self.show()
        self.register()
        self.triggered = False
        self.cur_regrow = 0

    def step(self):
        if self.triggered:
            self.cur_regrow -= 1
            if self.cur_regrow <= 0:
                self.show()
                self.register()
                self.triggered = False

    def is_walked(self, id):
        print("I'm walked")
        self.triggered = True
        self.rob.objects[self.gate_id].open()
        self.cur_regrow = self.regrow_time
        self.hide()
        self.unregister()


class GateObject(SquareObject):
    def __init__(self, id, data):
        SquareObject.__init__(self, id)
        self.triggered = False
        self.regrow_time = data['regrowTimeMax']
        self.cur_regrow = 0

    def reset(self):
        self.show()
        self.register()
        self.triggered = False
        self.cur_regrow = 0

    def step(self):
        if self.triggered:
            self.cur_regrow -= 1
            if self.cur_regrow <= 0:
                self.show()
                self.register()
                self.triggered = False

    def open(self):
        self.triggered = True
        self.hide()
        self.unregister()
        self.cur_regrow = self.regrow_time
```

Now, we need to tell in our configuration file that we want to use the objects that we defined. To do so, let's update our config file.

Let's update our configuration file:

```
  # Here we declare that we want a python object with the id "gate"
physicalObjects[0].pytype = gate
physicalObjects[1].pytype = switch
physcialObjects[1].sendMessageTo = 0
```

Then, when we create our Pyroborobo object, we must pass a dictionary mapping the
pytype key to our object classes. The Controller classes comes from the previous tutorial

```python
rob = Pyroborobo.create("config/pywander_pyobj.properties",
                        controller_class=SimpleController,
                        world_model_class=PyWorldModel,
                        object_class_dict={'gate': GateObject, 'switch': SwitchObject})
rob.start()
rob.update(3000)
Pyroborobo.close()
```