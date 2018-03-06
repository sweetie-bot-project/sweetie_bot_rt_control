function [denc_fir] = syn_fir_dfilter(N, M, p, R, T) 
	% Derivative FIR filter synthethis
	%
	% fir = syn_fir_dfilter(N, M, p, R, T)
	% 
	% Parameters
	%     N --- FIR filter order,
	%     M --- polynomial degree,
	%     p --- prediction steps.
	%     R --- derivative order.
	%     T --- sampling time in seconds.
	% Output
	%     fir --- filter structure:
	%		N, M, p, R --- filter parameters,
	%		h --- impulse response,
	%
	if (p < 0)
		error('syn_fir_dfilter: incorrect parameters: p < 0');
	endif

	if (M < 1)
		error('syn_fir_dfilter: incorrect parameters: M < 1');
	endif

	if (R > M || R < 0)
		error('syn_fir_dfilter: incorrect parameters: R > M or R < 0');
	endif

	if (N < M + 1)
		error('syn_fir_dfilter: incorrect parameters: N < M + 1');
	endif

	G = zeros(M + 1, M + 1);
	f = ones(M + 1, 1);

	sum = [];

	for j = 0:(M - 1)

		sum = [sum; 0];
		for k = 0:(N - 1)
			sum(j + 1) = sum(j + 1) + (N - k - 1) ^ (j);
		endfor

	endfor

	for i = 0:M

		sum = [sum ; 0];
		for k = 0:(N - 1)
			sum(M + i + 1) = sum(M + i + 1) + (N - k - 1) ^ (M + i);
		endfor

		for j = 0:M

			G(i + 1, j + 1) = sum(j + 1 + i);

		endfor

		for r=1:R
			f(i + 1) = f(i + 1) * (i - r + 1);
		endfor

		f(i + 1) = -2 * f(i + 1) * (N - 1 + p) ^ (i - R);

	endfor

	lambda = G \ f
G*lambda
	h = zeros(1, N);
	for k = 0:(N - 1)
		for j = 0:M
			h(k + 1) = h(k + 1) + lambda(j + 1) * (N - k - 1) ^ j;
		endfor
		h(k + 1) = -0.5 * h(k + 1);
	endfor

	denc_fir.N = N;
	denc_fir.M = M;
	denc_fir.p = p; 
	denc_fir.R = R;
	denc_fir.h = h / T ^ R; 
	denc_fir.sys = filt(denc_fir.h, 1, T);
end
