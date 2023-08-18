%% Data calibrated
acc_calib = zeros(178,3);
b = [0.026200; 0.031824;    -0.002348];
A_inv = [ 1.007368   0.000527    -0.004488;
          0.000527   1.006978     0.001624;
         -0.004488   0.001624     0.995385];

for i=1:178
    a_r = table2array(acc_raw(i,:)).';
    acc_calib(i,:) = A_inv * (a_r - b);
end

%%
p = plot3(acc_raw.acc_x, acc_raw.acc_y,acc_raw.acc_z,'.','MarkerSize',10);
p.Color = "red";
grid on
hold on
c = plot3(acc_calib(:,1), acc_calib(:,2), acc_calib(:,3),'.','MarkerSize',10);
c.Color = "blue";
xlim([-1.2 1.2])
ylim([-1.2 1.2])
zlim([-1.2 1.2])
lgd = legend('Datos en crudo','Datos calibrados');
lgd.Position = [0.75, 0.8, 0.08 0.07];
lgd.FontSize = 12;
%set(icons(3), 'MarkerSize',30);
xlabel('x');
ylabel('y');
zlabel('z');
view([90 0])
width=8*0.55;
height=6*0.55;
set(gcf,'units','inches');
set(gcf,'position',[0 0 width height]);
set(gcf,'PaperPosition',[0 0 width height]);
set(gcf,'PaperSize',[width height]);
set(gca, 'FontName', 'Times New Roman')
print('fig_graf','-dpdf')