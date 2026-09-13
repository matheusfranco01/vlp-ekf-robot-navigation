import numpy as np
from scipy.fft import rfft, rfftfreq, irfft

def FFT_filter(ilu_array:np.ndarray, sf:float, modulation_frequencies:float) -> np.ndarray:
    """
    Aplica FFT para filtrar o sinal de iluminância, mantendo apenas as componentes correspondentes às frequências de modulação.
    """

    # Garante um sinal 1D, que é o caso do grid com um único ponto.
    ilu_array = np.asarray(ilu_array, dtype=float).reshape(-1)

    # Configurações para a FFT
    N = 2000  # Número de pontos na FFT
    Ts = 1.0 / sf  # Período de amostragem

    # Remoção do nível DC do sinal
    ilu_array = ilu_array - np.mean(ilu_array)

    # Cálculo da FFT
    fft_results = rfft(ilu_array, n=N)
    frequencies = rfftfreq(N, d=Ts)

    # Filtrando o sinal: um valor RMS por frequência de modulação
    filtered_signal = np.zeros(len(modulation_frequencies), dtype=float)

    for i, f in enumerate(modulation_frequencies):
        idx = np.argmin(np.abs(frequencies - f))
        filtered_spectrum = np.zeros_like(fft_results)
        filtered_spectrum[idx] = fft_results[idx]
        time_signal = irfft(filtered_spectrum, n=N)
        filtered_signal[i] = np.sqrt(np.mean(np.square(time_signal)))

    return filtered_signal
