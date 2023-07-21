function [x_psd, y_psd] = fnPSD(time, signal, varargin)
    
    p = inputParser();
    addParameter(p, 'HighPassFreq', 3, @isnumeric); % high pass frequency in Hz, 0 to disable
    addParameter(p, 'HighPassOrder', 8, @isnumeric); % high pass butterworth order, affects minimum signal length
    parse(p, varargin{:});
    HighPassFreq = p.Results.HighPassFreq;
    HighPassOrder = p.Results.HighPassOrder;

    input_is_nan = isnan(time) | isnan(signal);
    if any(input_is_nan)
        i_seg_start = find(diff([1, input_is_nan]) < 0);
        i_seg_end = find(diff([input_is_nan, 1]) > 0);
        wgt = nan(numel(i_seg_start),1);
        for iSeg = 1:numel(i_seg_start)
            t_local = time(i_seg_start(iSeg):i_seg_end(iSeg));
            y_local = signal(i_seg_start(iSeg):i_seg_end(iSeg));
            wgt(iSeg) = t_local(end) - t_local(1); % time in this segment
            [x_psd_local, y_psd_local] = generate_psd_inner(t_local, y_local, HighPassFreq, HighPassOrder);
            x_lout(iSeg,:) = x_psd_local; %#ok<AGROW>
            y_lout(iSeg,:) = y_psd_local; %#ok<AGROW>
        end
        x_psd = mean(x_lout, 1, 'omitnan');
        y_psd = sum((y_lout .* wgt), 1, 'omitnan') / sum(wgt); % average back together
        % debugging plot:
        % figure(101); hold off; plot(x_lout', y_lout'); hold on; xlim([0, 15]);
        % plot(x', y', 'k:', 'LineWidth', 1.5); grid on;
    else
        [x_psd, y_psd] = generate_psd_inner(time, signal, HighPassFreq, HighPassOrder);
        x_psd = x_psd'; y_psd = y_psd'; % adjust to row vectors
    end

end

function [x_local, y_local] = generate_psd_inner(t_local, y_local, HighPassFreq, HighPassOrder)
    overlap = []; % 50 %
    nfft = 4096;
%     nfft = [];
    min_t = 0.4; % 0.4 s hard cutoff, below that is pointless
    t_extrema = max(t_local)-min(t_local);
    windowsize = min(t_extrema, 4); % seconds, width of hanning window

    bHighPass = HighPassFreq > 0;
    if ~bHighPass
        HighPassOrder = 1;
    end

    lineartime = min(t_local):0.002:max(t_local);
    if t_extrema < min_t || numel(lineartime) <= HighPassOrder*3
        x_local = nan(nfft/2+1, 1);
        y_local = x_local;
    else
        signal = spline(t_local, y_local, lineartime);
        dt = mean(diff(lineartime));
        sfreq = 1/dt;
        if bHighPass
            [b , a] = butter(HighPassOrder , 2 * HighPassFreq / sfreq , 'high');
            fsignal = filtfilt(b, a, signal);
        else
            fsignal = signal;
        end
        [y_local , x_local] = pwelch(fsignal, hanning(round(windowsize*sfreq)) , overlap, nfft, sfreq);
    end
end