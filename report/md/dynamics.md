# The dynamics of the system

## Summary

We started using a simple model, the damped string model. It exhibits an important property of the system : the relaxation and max stimulation frequencies are different, making the tuning of the system more complex.

The model predicts a first order response on the amplitude of each mode, when the drive frequency is _exactly_ the resonant frequency, and predicts beatings when the difference is small which is coherent with what is seen experimentally.

_However, this simple model, which parameters seem too dependent on the environmental conditions of the instrument, was not used in the experimental control._

## Calculations 

### The homogeneous equation

We started with a very simple model of this instrument : the damped string, in one dimension.

Let $u(x,t)$ be the height of the string above its resting position at position $x$ and time $t$.  We can write the damped string equation $$u_{tt} - c^2 u_{xx} + \lambda u_t = 0 \quad(1)$$. with boundary conditions $u(0) = u(L) = 0$. Supposing $u$ is harmonic and can be written as $u(x,t) = f(x) \, g(t)$, we find $f''(x) = k \,f(x)$ and $g''(t) + \lambda g'(t) = c^2 k g(t)$. The asymptotic stability imposes $k<0$, hence $k = - \alpha^2$

The boundary conditions impose $f(0)=f(L)=0$, hence $f(x) = A \sin(\alpha_n x)$, with $\alpha_n L = n \pi$. e.g. $\alpha_n = \frac{n\pi}{L}$. The solutions of  $g''(t) + \lambda g'(t) + c^2\alpha_n^2 g(t) = 0$ are $t\mapsto B\exp(\beta t)$ with $\beta\in\mathbb{C}$ and $B\in\mathbb{R}$. Re-injecting in the previous equation, we find $\beta^2 \exp(\beta t) + \lambda \beta \exp(\beta t) + c^2 \alpha_n^2 \exp(\beta t) = 0$ hence $\beta^2 + \lambda\beta + c^2\alpha_n^2 = 0$. The real system oscillates with little friction, so we can assume that $\lambda - 4c^2\alpha_n^2<0$.

Hence $$\beta = \frac{-\lambda\pm j\sqrt{4c^2\alpha_n^2 - \lambda^2}}{2}=\frac{-\lambda}{2} \pm j\sqrt{c^2\alpha_n^2 - \left(\frac{\lambda}{2}\right)^2}$$ 

Let $\omega_{0,n} = \frac{c \,n\,\pi}{L}$, thus $$\beta  = \frac{-\lambda}{2} \pm j \omega_{0,n}\sqrt{1 - \left(\frac{\lambda L}{2cn\pi}\right)^2}$$ . 

We will also call $$\omega_{u,n} = \omega_{0,n}\sqrt{1 - \left(\frac{\lambda L}{2cn\pi}\right)^2}$$ the pseudo-frequency of undriven oscillations.



Now solving for the general solution, we had  $f_n(x) = A \sin(\alpha_n x)$. We suppose $A = \frac{L}{\sqrt{2}}$ (since constants can be adjusted to boundary conditions later). Since $(f_n)_{n\in\mathbb{N^*}}$ is an orthonormal basis of the Hilbert space  $L^2(]0,L[)$, we can write  $$u = \sum_{n\in\mathbb{N^*}} g_n(t) f_n(x) = \sum_{n\in\mathbb{N^*}} g_n(t) \sin\left(\frac{n\pi}{L}x\right)$$.

The linearity of the wave equation and the orthogonality of the $(f_n)_{n\in\mathbb{N^*}}$ gives $g_n''(t) + \lambda g_n'(t) + c^2\alpha_n^2 g_n(t) = 0$ and so $g_n = B_n \exp(-\frac{\lambda}{2}) \sin(\omega_{l,n}t + \phi_n)$.

Hence the general form of the homogeneous equation :

$$u = \sum_{n\in\mathbb{N^*}} B_n \exp\left(-\frac{\lambda t}{2}\right) \sin(\omega_{l,n}t + \phi_n)\sin\left(\frac{n\pi}{L}x\right)$$



### Adding the actuator

Let us now consider the non-homogeneous damped string equation : 

$$u_{tt} - c^2 u_{xx} + \lambda u_t = h(x,t) \quad(2)$$.

where $h : ]0,L[\times \mathbb{R}^+ \to \mathbb{R}$ is a smooth function that can be written as $h(x,t) = d(x)s(t)$. 

Projecting $h$ on the $(f_n)_{n\in\mathbb{N^*}}$ basis gives  $$h(x, t) =  \sum_{n\in\mathbb{N^*}} s(t) \,c_n\sin\left(\frac{n\pi}{L}x\right)$$, where $c_n = \frac{\sqrt{2}}{L}<f_n,d>$ !!!!!!! vérifier les constantes. 

The $c_n$ scalars represent the spatial distribution of the force, while $s(t)$ represents its time dependent variations.

Equation $(2)$ yields  $$\sum_{n\in\mathbb{N^*}} \left(g_n''(t) + \lambda g_n'(t) + c^2\alpha_n^2 g_n(t) \right) \sin\left(\frac{n\pi}{L}x\right) =  \sum_{n\in\mathbb{N^*}} s(t)\,k_n \sin\left(\frac{n\pi}{L}x\right)$$. 

Hence $g_n''(t) + \lambda g_n'(t) + c^2\alpha_n^2 g_n(t) =  k_n \, s(t)$, since the $(f_n)_{n \in \mathbb{N*}}$ are orthogonal.

In complex notation, we get

$(j\omega)^2\underline{g} + j\omega\lambda\underline{g} + \omega_{0,n}^2\underline{g} = k_n \underline{s}$  hence $$\underline{g} = \underline{s}\frac{k_n}{\omega_{0,n}^2-\omega^2 + j\omega\lambda}$$

This is the typical behaviour of a second order low pass filter with a resonant frequency  $\omega = \omega_{0,n}$.



The simulations of cylindrical permanent magnets and the magnetic scans are coherent with a simplification of the force of the type : 

$$d(x) =  \bigg( \begin{matrix}  0 \text{ if } x \notin [x_0 -\frac{\delta x}{2},x_0 +\frac{\delta x}{2}] \\ \frac{d_{max}}{(\frac{\delta x}{2})^2}(x-(x_0 -\frac{\delta x}{2}))(x - (x_0 +\frac{\delta x}{2})) \text{ otherwise}\end{matrix}$$.

![force_aimant](../img/force_aimant.svg)

This definition is that of a parabola centered around $x_0$, with a max height of $d_{max}$ and a distance between its roots of $\delta x$. The function is zero outside of $[x_0 -\frac{\delta x}{2},x_0 +\frac{\delta x}{2}]$, hence out of the bell region. We assume this segment to be part of $[0,L]$.

We can now calculate $c_n$ : 

 $$k_n = \frac{\sqrt{2}}{L} < f_n, d> \;= \sqrt{\frac{2}{L}}\int_0^L \sin\left(\frac{n \pi x}{L}\right)d(x) \; \text{d}x =  \int_{x_0 - \frac{\delta x}{2}}^{x_0 + \frac{\delta x}{2}}\sin\left(\frac{n \pi x}{L}\right)d(x) \; \text{d}x  \\ = c \left( \frac{2}{\alpha_n^3}\left(\delta x\alpha_n\cos\left(\frac{\delta x \alpha_n}{2}\right) - 2 \sin\left(\frac{\delta x \alpha_n}{2}\right)\right)\sin(\alpha_n x_0)\right)$$

!!!!! constante $c$ à re-préciser.

For small actuators, this result is close to the one we would get from the single point model : $k_n \approx c' \sin(\alpha_n x_0)$ .





Cherchons la réponse d'un mode à un "échelon de sinus" c'est à dire une fonction de la forme $t\mapsto u_0(t)\; A\sin(\omega t)$ où $t\mapsto u_c(t)$ est la fonction de Heaviside définie par $u_c(t) =\bigg( \begin{matrix} 0 \text{ si } t<c  \\ 1 \text{ si } t \geq c\end{matrix}$

On a donc : 

$\ddot{q} + \lambda  \dot{q} + \omega_0^2q = u_0(t) \; A \sin(\omega t)$

Donc dans le domaine de Laplace, le système étant au repos au temps $t<0$,

$$ s^2Q(s) + \lambda  sQ(s) + \omega_0^2Q = A \frac{\omega}{s^2 + \omega^2}$$ ,

C'est à dire  $$Q(s) = \frac{A}{s^2 + \lambda s + \omega_0^2} \frac{\omega}{s^2 + \omega^2}$$



### Testing the simple model

We collected a set of data representing the system response to different step inputs.

We used the Bela to inject a signal of the type : 

$$s(t) = r(t)\left(\sum_{i=1}^5 a_i\sin(w_i (t-1))\right)$$ where $t\mapsto r(t)$ is a ramp signal defined as

$r(t) = \Bigg( \begin{matrix} 0 \text{ if } t<1  \\ (t-1)/D \text{ if } 1\leq t \leq 1+\epsilon \\  1  \text{ if } t>1+\epsilon \end{matrix}$ And phases were chosen so as to limit the high frequency click at $t=1$.

The $a_i$ amplitudes were chosen to study both single frequency response and multimodes excitation.

The below figure shows the system response to a `0.5` step input.

!!!!!

Seeing that the response is far from that of a first order system, we tried to tune the string more carefully.

### The issue of tuning the string

Tuning the string to an exact frequency is a difficult process : as seen previously, the free oscillation frequency differs from the resonance frequency, and this relationship depends on the damping of the system. The string being attached to other mechanical parts (the piezo bridge, the nut and the tuning mechanism, and the wood itself), the damping is a function of the entire instrument geometry. Furthermore, the surface on which the instrument is played also dissipates energy, as it radiates sound.

To mitigate the surface dependancy, we layed the instrument on thick foam legs, dramatically reducing the transmitted vibration.

We used stroboscopy to tune the string : we injected a pure sine at a fixed frequency into the actuator, and then used an LED array driven at the same frequency to see the actual movement of the string.

We also reduced the illumination time to a tenth of the period the fundamental : this reduced motion blur and allowed finer tuning.

We were constantly confronted with a kind of hysterisis : the string would start its vibration when driven at a frequency close to $f_0$, but would still vibrate, sometimes with a higher amplitude, when driven at a frequency further off from $f_0$ .

This effect can be seen in the next video : 

!!!!!!

Our method for tuning was to regularly stop the string and wait for the amplitude to settle, while checking the frequency with the strobe. 



## Bibliography

[**Transient Oscillator Response**](https://farside.ph.utexas.edu/teaching/315/Waveshtml/node15.html)