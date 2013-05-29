
require 'rubygems'
require 'midilib'

require 'serialport'

# midi 0 -> C_0 -> 16 HZ
tone_map_hz = [
  16.35, 17.32,  18.35,  19.45,  20.60,  21.83,  23.12,  24.50,
  25.96,  27.50,  29.14,  30.87,  32.70,  34.65,  36.71,  38.89,
  41.20,  43.65,  46.25,  49.00,  51.91,  55.00,  58.27,  61.74, 
  65.41,  69.30,  73.42,  77.78,  82.41,  87.31,  92.50,  98.00, 
 103.83,  110.00,  116.54,  123.47,  130.81,  138.59,  146.83,  155.56,
 164.81,  174.61,  185.00,  196.00,  207.65,  220.00,  233.08,  246.94,
 261.63,  277.18,  293.66,  311.13,  329.63,  349.23,  369.99,  392.00
]


filename = 'imperial.mid'

seq = MIDI::Sequence.new

File.open filename, 'rb' do |file|
  seq.read file 
end


#
# Open serial port
#
fdrive = SerialPort.new '/dev/ttyUSB0', 9600



# play 
seq.each do |track|
  track.each do |e|
    if e.is_a? MIDI::NoteOn 
      sleep e.delta_time / 1000.0
      puts "NoteOn! #{e.note}"
    
      transposed = e.note - (4*8) - 4
      freq = tone_map_hz[transposed] 
      freq = 420 if freq.nil?

      fdrive.write "200 #{freq.round}\n"
      puts " -> #{freq.round} Hz"

      sleep (e.off.delta_time / 1000.0)

      puts "NoteOff."
      
      fdrive.write "200 0\n"
    end
  end
end



