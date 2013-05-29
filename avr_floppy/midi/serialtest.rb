
require 'serialport'


port = SerialPort.new '/dev/ttyUSB0', 9600


100.times do 
  tone = 10 + rand(200)
  puts "Setting tone to: #{tone}"
  port.write "200 #{tone}\n"
  sleep 0.25

end


