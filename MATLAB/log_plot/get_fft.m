function [f, P1] = get_fft(raw_data, Ts)
    L = length(raw_data);
    sampling_rate = 1/Ts;
    Raw_FFT = fft(raw_data);
    P2 = abs(Raw_FFT / L);
    P1 = P2(1:L/2 + 1);
    P1(2:end - 1) = 2 * P1(2:end - 1);
    f = sampling_rate / L * (0:(L/2));
end