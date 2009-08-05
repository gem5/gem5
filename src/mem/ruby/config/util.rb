
def log_int(n)
  assert(n.is_a?(Fixnum), "log_int takes a number for an argument")
  counter = 0
  while n >= 2 do
    counter += 1
    n = n >> 1
  end
  return counter
end
