f = open('raw_data.tsv')
f.readline()  # пропускаем заголовки
fo = open('raw_data_.tsv', 'w')
fo.write('ax\tay\taz\tax_c\tay_c\taz_c\tgx\tgy\tgz\tbrake\n')

for l in f.readlines():
  (ax, ay, az, ax_c, ay_c, az_c, gx, gy, gz, brake) = l.split("\t")
  ax = int(round(int(ax) / 10, 0))
  ay = int(round(int(ay) / 10, 0))
  az = int(round(int(az) / 10, 0))
  ax_c = int(round(int(ax_c) / 10, 0))
  ay_c = int(round(int(ay_c) / 10, 0))
  az_c = int(round(int(az_c) / 10, 0))
  gx = int(round(int(gx) / 100, 0))
  gy = int(round(int(gy) / 100, 0))
  gz = int(round(int(gz) / 100, 0))
  fo.write(f'{ax}\t{ay}\t{az}\t{ax_c}\t{ay_c}\t{az_c}\t{gx}\t{gy}\t{gz}\t{brake}')
  
f.close()
fo.close()