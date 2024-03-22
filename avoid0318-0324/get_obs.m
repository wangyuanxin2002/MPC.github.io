function [Nobs, X_obstacle, Y_obstacle, dX_obstacle, dY_obstacle] = get_obs(t)
    Nobs = 5;
    X_obstacle=zeros(Nobs,1);
%     X_obstacle=(30:50)';
%     X_obstacle(2)=30;
    X_obstacle=X_obstacle +30;

    Y_obstacle=zeros(Nobs,1);
    Y_obstacle=Y_obstacle + 3;
    dX_obstacle=zeros(Nobs,1);
    dX_obstacle = dX_obstacle + 0;
    dY_obstacle=zeros(Nobs,1);
    dY_obstacle = dY_obstacle - 0.6;
%     Y_obstacle(2)=1;
end
