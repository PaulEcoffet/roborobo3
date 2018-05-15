import numpy as np
from pyevo import getES

es = getES('mulambda', np.zeros(10), 0.01, 10, [-1, 1], 1000, '')
np.set_printoptions(2)


while not es.stop():
    print("*"*30)
    sol = es.ask()
    print(sol)
    print(np.sum(sol, axis=1))
    es.tell(sol, np.sum(sol, axis=1))
