
ruby_cfg_file = nil
$stderr.puts $*.inspect
for i in 0..$*.size-1 do
  if $*[i] == "-r" # ruby config file
    i += 1
    ruby_cfg_file = $*[i]
    break
  end
end

require ruby_cfg_file

RubySystem.generateConfig
