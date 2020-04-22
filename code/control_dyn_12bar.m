function control_dyn_12bar(FAx,FAy,FEx,FEy,FFx,FFy,FLy,FPy,...
                            acc2x,acc2y,acc3x,acc3y,acc4x,acc4y,acc5x,acc5y,acc6x,acc6y,acc7x,acc7y,acc8x,acc8y,acc9x,acc9y,acc10x,acc10y,acc11x,acc11y,acc12x,acc12y,...
                            m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12,...
                            t)

    Fshake_x_1 = FAx + FFx + FEx;
    Fshake_x_2 = acc2x*m2 + acc3x*m3 + acc4x*m4 + acc5x*m5 + acc6x*m6 + acc7x*m7 + acc8x*m8 + acc9x*m9 + acc10x*m10 + acc11x*m11 + acc12x*m12;
    
    Fshake_y_1 = FAy + FFy + FEy + FLy + FPy;
    Fshake_y_2 = acc2y*m2 + acc3y*m3 + acc4y*m4 + acc5y*m5 + acc6y*m6 + acc7y*m7 + acc8y*m8 + acc9y*m9 + acc10y*m10 + acc11y*m11 + acc12y*m12;

    % Plot control Fshake in X direction
    figure
    plot(t,Fshake_x_1 - Fshake_x_2)
    ylabel('Fshake_x [N]')
    xlabel('t [s]')
    title('error of shaking forces in x direction')
    
    % Plot control Fshake in y direction
    figure
    plot(t,Fshake_y_1 - Fshake_y_2)
    ylabel('Fshake_y [N]')
    xlabel('t [s]')
    title('error of shaking forces in y direction')
    
end