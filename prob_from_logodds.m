function p = prob_from_logodds(L)
    p = 1 - 1 ./ (1 + exp(L)); % 0=free(white), 1=occupied(black), 0.5=unknown(grey)
end

