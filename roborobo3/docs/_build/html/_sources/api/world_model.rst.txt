.. currentmodule:: pyroborobo

World Models
============

RobotWorldModel
---------------

This class is the binding from c++ to python of the RobotWorldModel class. It has less functionalities than :class:`~pyroborobo.PyWorldModel`. If you plan to extend the World Model of your agent, you should intherit from :class:`~pyroborobo.PyWorldModel`.


.. autoclass:: RobotWorldModel
    :members:

PyWorldModel
------------

Documentation
*************

..
   _This class is the binding from c++ to python of the RobotWorldModel class. It has less functionalities than :class:`~pyroborobo.PyWorldModel`. If you plan to extend the World Model of your agent, you should intherit from :class:`~pyroborobo.PyWorldModel.

.. autoclass:: pyroborobo.PyWorldModel
    :members:
    :inherited-members:

Template
********

If you want to extend PyWorldModel, you can use this template:

.. code-block:: python

    class MyWorldModel(PyWorldModel):
        def __init__(self):
            super().__init__()

        def reset(self):
            super().reset()
