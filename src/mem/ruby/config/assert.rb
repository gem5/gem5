#!/usr/bin/env ruby

class AssertionFailure < RuntimeError
  attr_reader :msg, :output
  def initialize(message, out=nil)
    @msg = message
    @output = out
  end
end

class NotImplementedException < Exception
end

def assert(condition,message)
  unless condition
    raise AssertionFailure.new(message), "\n\nAssertion failed: \n\n    #{message}\n\n"
  end
end
