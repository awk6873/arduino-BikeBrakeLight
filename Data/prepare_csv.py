f = open('raw_data_lbl_2022-09-29.csv')
f.readline()  # пропускаем заголовки
fo = open('raw_data_edge_2022-09-29.csv', 'w')
fo.write('timestamp,ax,ay,az,gx,gy,gz,brake\n')

timestamp = 0
for l in f.readlines():
  (ax, ay, az, gx, gy, gz, mx, my, mz, delta_t, brake) = l.split(",")
  brake = int(brake) * 3000
  timestamp = int(timestamp + int(delta_t) / 1000)
  #print(timestamp)
  fo.write(f'{timestamp},{ax},{ay},{az},{gx},{gy},{gz},{brake}\n')
  
f.close()
fo.close()