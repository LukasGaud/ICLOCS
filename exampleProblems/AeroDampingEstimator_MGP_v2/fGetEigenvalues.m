function [lambda, eigVec] = fGetEigenvalues(problem, solution)

data = problem.data.auxData;


ms = data.P.ms;
mu = data.P.mu;
ct = data.P.ct;
kt = data.P.kt;

for i = 1:size(solution.U(:,1), 1)
    ca = solution.U(i,1);
    ka = solution.U(i,2);
    cs = solution.U(i,3);
    ks = solution.U(i,4);

    A = [
        0, 1, 0, 0;
        (-ks./ms - ka./ms), (-cs./ms - ca./ms), (ks./ms), (cs./ms);
        0, 0, 0, 1;
        (ks./mu), (cs./mu), (-ks./mu - kt./mu), (-cs./mu - ct./mu)];
    [V, D] = eig(A);
%     if i == 1
        [~,ind] = sort(real(diag(D)));
        L = diag(D(:, ind));
        V0 = V(:, ind);
        X = V(:, ind);
%     else
%         for j = 1:size(V0,2)
%             v = V(:,j);
%             mag = v'*V0;
%             [~, n] = max(abs(mag));
%             X(:, j) = V(:, n);
%             L(:, j) = D(:, n);
%         end
%     end
    lambda(:,i) = L;
    eigVec{i} = X;
end