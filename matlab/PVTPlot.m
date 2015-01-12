function y=PVTPlot(fig_id, time, pos, vel, acc, v_max, a_max, t1, t2, t3, t4, t5)
  figure(fig_id);
  subplot(3,1,1);
  plot([t2, t2], 1.1*[min(pos), max(pos)], 'g', [t3, t3], 1.1*[min(pos), max(pos)], 'g', [t4, t4], 1.1*[min(pos), max(pos)], 'g', [t5, t5], 1.1*[min(pos), max(pos)], 'g');
  hold on
  plot(time, pos, 'b');
  xlabel ('time [s]');
  ylabel ('position [rad]');
  subplot(3,1,2);
  plot([t2, t2], 1.1*[min([vel,-v_max]), max([vel,v_max])], 'g', [t3, t3], 1.1*[min([vel,-v_max]), max([vel,v_max])], 'g', [t4, t4], 1.1*[min([vel,-v_max]), max([vel,v_max])], 'g', [t5, t5], 1.1*[min([vel,-v_max]), max([vel,v_max])], 'g');
  hold on
  plot([t1,t5], [v_max,v_max], 'r', [t1,t5], [-v_max,-v_max], 'r');
  plot(time, vel, 'b');
  xlabel ('time [s]');
  ylabel ('velocity [rad/s]');
  subplot(3,1,3);
  plot([t2, t2], 1.1*[min([acc,-a_max]), max([acc,a_max])], 'g', [t3, t3], 1.1*[min([acc,-a_max]), max([acc,a_max])], 'g', [t4, t4], 1.1*[min([acc,-a_max]), max([acc,a_max])], 'g', [t5, t5], 1.1*[min([acc,-a_max]), max([acc,a_max])], 'g');
  hold on
  plot([t1,t5], [a_max,a_max], 'r', [t1,t5], [-a_max,-a_max], 'r');
  plot(time, acc, 'b');
  xlabel ('time [s]');
  ylabel ('acceleration [rad/s/s]');
  y=0;
end
