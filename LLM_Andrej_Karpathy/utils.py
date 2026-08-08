import numpy as np
from micrograd.engine import Value
import micrograd
import torch

def generate_data(x_raw: np.array, 
                  y_raw: np.array, 
                  train: float, 
                  val: float, 
                  test: float) -> tuple(tuple(np.array, np.array), tuple(np.array, np.array), tuple(np.array, np.array)):

    """ write a custom function to generate training data """

    # 1. Takes x_raw, and y_raw
    # 2. Construts and returns train, test and val datasets

    """
    Validation data can be used for:
    - try different learning rates, layer sizes, regularization strengths, etc. Compare their validation losses.
      Pick the combo with lowest val loss. This is a search loop around your training loop.
    - Model selection (architecture search) — if you're debating between [4,4,1] vs [8,4,1] vs [16,8,4,1], 
     you'd train each on the same train set, compare validation loss, and pick the winner. Then evaluate that winner once on the test set.
    """

    assert train + val + test == 1.0

    n = len(x_raw)

    # create randomized indexes
    idxs = np.random.permutation(n)

    # compute indexes
    tr_idx = int(n*train)
    tr_idxs = idxs[:tr_idx]
    ts_idx = int(n*test)
    ts_idxs = idxs[tr_idx: tr_idx+ts_idx]
    vl_idxs = idxs[tr_idx+ts_idx:]

    x_train, y_train = x_raw[tr_idxs], y_raw[tr_idxs]
    x_test, y_test = x_raw[ts_idxs], y_raw[ts_idxs]
    x_val, y_val = x_raw[vl_idxs], y_raw[vl_idxs]

    x_train = [Value(each) for each in x_train]
    y_train = [Value(each) for each in y_train] #[(x_tr)**2 for x_tr in x_train]

    x_val = [Value(each) for each in x_val]
    y_val = [Value(each) for each in y_val]

    x_test = [Value(each) for each in x_test]
    y_test = [Value(each) for each in y_test]

    return (x_train, y_train), (x_test, y_test), (x_val, y_val)

def SGD(p: micrograd.engine.Value, 
        lr: float):

    return lr * p.grad

def momentum(v: float, 
             p: micrograd.engine.Value, 
             beta: float):

    # beta is like a remembering factor, to ensure finiteness, beta must be less than 1.0
    assert beta < 1.0
    v1 = beta*v + p.grad # proxy for gradient, history/accumulation of gradients, with latest gra
    return v1

def rmsprop(s: float, 
            p: micrograd.engine.Value, 
            beta: float) -> float:

    assert beta < 1.0
    s1 = beta * s + (1-beta)*(p.grad**2)
    return s1

def adam(v_adam: float, 
         s_adam: float, 
         p: micrograd.engine.Value, 
         beta1: float, 
         beta2: float, 
         epoch_num: int) -> tuple[float, float, float, float]:

    assert beta1 < 1.0 and beta2 < 1.0

    v1_adam = beta1*v_adam + (1-beta1)*p.grad
    s1_adam = beta2*s_adam + (1-beta2)*p.grad**2

    # bias correction: (initial velocities and squared gradients are squared towards 0)
    v1_adam_hat = v1_adam/(1- beta1**(epoch_num+1))
    s1_adam_hat = s1_adam/(1 - beta2**(epoch_num+1))

    return v1_adam, s1_adam, v1_adam_hat, s1_adam_hat

def tanhValue(inp: Value):

    """ Implement a tanH function for the micrograd values data type """

    inpData = torch.Tensor([inp.data]).double()
    return Value(torch.tanh(inpData).item())

def sineValue(inp: Value):

    """ Implement a sine function for micrograd values data type """

    inpData = inp.data
    return Value(np.sin(inpData))