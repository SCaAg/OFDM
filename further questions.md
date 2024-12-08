### 1) Based on the analysis presented above, can you derive the maximum target range and velocity for the OFDM sensing system as a function of system parameters?
The matrix **F** is the periodogram's input data. The periodogram needs to be extended to two dimensions as this is two-dimensional, and the solution put forward is below:
$$
\text{Per}_F(n, m) = \frac{1}{N M} \left| \sum_{k=0}^{N_{\text{Per}}-1} \sum_{l=0}^{M_{\text{Per}}-1} (F)_{k, l} e^{-j 2\pi \frac{l m}{M_{\text{Per}}}} e^{j 2\pi \frac{k n}{N_{\text{Per}}}} \right|^2 = \frac{1}{N M} \left| \text{CPer}_F(n, m) \right|^2
$$

where $(F)_{k, l} = \frac{(F_{\text{Rx}})_{k, l}}{(F_{\text{Tx}})_{k, l}}$),$F_\text{Tx}$​ is the transmitted OFDM signal,$F_\text{Rx}$​ is the received OFDM signal. Sinusoids in **F** will lead to a peak in $\text{Per}_F(\hat{n}, \hat{m})$​. Thus, the peaks must be first detected by the algorithms based on the periodogram. Thus, **F** has a row-wise oscillation of frequency $\hat {Ω}_v=\frac {2π \hat{m}}{M_ \text{Per}}$and a column-wise oscillation of frequency$\hat {Ω}_d=\frac {2π \hat{n}}{N_ \text{Per}}$corresponds to a peak value. This corresponds to a target estimate of relative velocity and distance by:
$$\hat{d} = \frac{1}{2} t c_0 = \frac{\hat{\Omega}_d c_0}{2 (2\pi) \Delta f} = \frac{\hat{n} c_0}{2 \Delta f N_{\text{Per}}}$$
$$\hat{v} = \frac{f_d c_0}{2 f_c} = \frac{\hat{\Omega}_v c_0}{2 (2\pi) f_c T_0} = \frac{\hat{m} c_0}{2 f_c T_0 M_{\text{Per}}}$$
where:
- $c_0$​ is the speed of light,
- $\Delta f$ is the sub-carrier spacing,
- $f_c$​ is the carrier frequency,
- $T_0$​ is the OFDM symbol duration,
- $N_ \text{Per}$​ is the number of rows per two-dimensional periodogram, equals to $N_\text{FFT}$ 
- $M_ \text{Per}​$ is the number of columns per two-dimensional periodogram, equals to $M_\text{FFT}$ .[1]
According to the definition:
$$0\leq |\hat{n}| \leq N_\text{Per}-1 ,\quad  0\leq |\hat{m}| \leq M_\text{Per}-1$$
then:
$$\hat{d}_\text{max}\approx\frac {c_0}{2\Delta f}$$
$$\hat{v}_\text{max}\approx\frac {c_0}{2f_c T_0}$$
### 2) Based on the analysis presented above, can you derive the target range and velocity resolution for the OFDM sensing system as a function of system parameters? Can we improve the resolution by using a larger FFT/IFFT computation size?
Obviously:
$$ 0\leq\hat{d}\leq \hat{d}_\text{max}\approx\frac {c_0}{2\Delta f}$$
$$ 0\leq\hat{v}\ll \hat{v}_\text{max}\approx\frac {c_0}{2f_cT_0}$$
For the resolution:
$$\hat{d}_\text{re}=\frac {c_0}{2\Delta fN_\text{FFT}}$$
$$\hat{v}_\text{re}=\frac {c_0}{2f_cT_0M_\text{FFT}}$$
We can improve the resolution by using a larger FFT/IFFT computation size
### 3) How does the guard interval (CP length) affect its sensing performance?

Ncp = 16
![[Pasted image 20241208170032.png]]
Ncp = 32
![[Pasted image 20241208165957.png]]
Longer CP means means a more accurate perception of speed. However, excessively long CP will induce more errors to the decoding of the message.
### 4) For the monostatic sensing, it is assumed that the transmitted symbols are totally known by the BS and hence it can be perfectly removed from the received signal. For bi-static sensing, the data needs to be decoded first. Examine the decoding error on the sensing performance
Find in DSPOFDMf4
**The sensing performance is better with the increasement of SNR**


### References:
[1]Zhang H, Zhang Y, Qi X, et al. Moving target detection based on OFDM radar[J]. The Journal of Engineering, 2019, 2019(19): 5605-5609.