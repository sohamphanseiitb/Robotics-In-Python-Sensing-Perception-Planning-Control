
import numpy as np
from utils import SGD, momentum, rmsprop, adam
from micrograd.engine import Value
from micrograd.nn import Module, Neuron, Layer, MLP


def square_loss(y_pred: list, y_train: list):

    loss = sum((yp - yt)**2 for yp, yt in zip(y_pred, y_train)) / len(y_train)

    return loss


def simple_train(x_train: list,
                 y_train: list,
                 x_val: list,
                 y_val: list,
                 MLP1: MLP, 
                 lr: float = 0.005, 
                 beta: float = 0.9, 
                 epochs: int = 3000, 
                 epsilon: float = 1e-10, 
                 beta1: float = 0.9, 
                 beta2: float = 0.999, 
                 algo: str = "grad",
                 lossFunc: str="square-loss"
                 ) -> tuple[np.array[float], MLP]:

    """
    MLP: Neural Net
    lr: learning rate
    beta: decay rate for momentum
    epochs: number of epochs to train for
    epsilon: small positive value
    beta1: decay rate for momentum for adam
    beta2: decay rate for squared gradients for adam
    algo: algorithm to train with, "grad", "momentum", "rmsprop", and "adam"
    """

    # define loss store:
    loss_store = np.zeros(epochs)
    val_loss_store = np.zeros(epochs)

    # define velocity and squared gradients
    v = {p: 0.0 for p in MLP1.parameters()}
    s = {p: 0.0 for p in MLP1.parameters()}

    for epoch in range(epochs):

        # Forward pass
        y_pred = [MLP1([x]) for x in x_train]

        # MSE loss
        if lossFunc == "square-loss": 
            loss = square_loss(y_pred, y_train)

        # Zero gradients
        for p in MLP1.parameters():
            p.grad = 0.0

        # do backward pass
        loss.backward()

        # Gradient descent
        for p in MLP1.parameters():
            if algo=="grad":
                p.data -= SGD(p, lr) #lr * p.grad # Stochastic Gradient Descent
            elif algo=="momentum":
                v[p] = momentum(v[p], p, beta)
                p.data -= lr*v[p]
            elif algo=="rmsprop":
                s[p] = rmsprop(s[p], p, beta)
                p.data -= (lr/(np.sqrt(s[p]) + epsilon))*p.grad
            elif algo=="adam":
                v[p], s[p], v_hat, s_hat = adam(v[p], s[p], p, beta1, beta2, epoch)
                p.data -= lr*v_hat/(np.sqrt(s_hat) + epsilon)
            else:
                raise ValueError(f"Unknown algorithm: {algo}")

        # store training loss
        loss_store[epoch] = loss.data

        # store validation loss
        y_val_pred = [MLP1([x]) for x in x_val]
        if lossFunc == "square-loss":
            val_loss = square_loss(y_val_pred, y_val) #sum((yp - yt)**2 for yp, yt in zip(y_val_pred, y_val)) / len(y_val)
        val_loss_store[epoch] = val_loss.data


    return [loss_store, val_loss_store, MLP1]