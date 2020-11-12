# Makefile-based Testing Infrastructure
# Scott Beamer, 2015



# Shared Components ----------------------------------------------------#
#-----------------------------------------------------------------------#

# Dependencies are the tests it will run
test-all: test-build test-generate test-load test-verify

# Does everthing, intended target for users
test: test-score
	@if $(MAKE) test-score | grep FAIL > /dev/null; \
		then exit 1; \
	fi

# Sums up number of passes and fails
test-score: test-all
	@$(MAKE) test-all | cut -d\  -f 2 | grep 'PASS\|FAIL' | sort | uniq -c

# Result output strings
PASS = \033[92mPASS\033[0m
FAIL = \033[91mFAIL\033[0m

test/out:
	mkdir -p test/out

# Need to be able to build kernels, if this fails rest not run
test-build: all
	@echo " $(PASS) Build"



# Graph Generation/Building/Loading ------------------------------------#
#-----------------------------------------------------------------------#

# Since all implementations use same code for this, only test one kernel
GENERATE_KERNEL = bfs

# Built-in synthetic graph generators
test-generate: test-generate-g10 test-generate-u10

test/out/generate-%.out: test/out $(GENERATE_KERNEL)
	./$(GENERATE_KERNEL) -$* -n0 > $@

.SECONDARY: # want to keep all intermediate files (test outputs)
test-generate-%: test/out/generate-%.out
	@if grep -q "`cat test/reference/graph-$*.out`" $<; \
		then echo " $(PASS) Generates $*"; \
		else echo " $(FAIL) Generates $*"; \
	fi

# Loading graphs from files
test-load: test-load-4.gr test-load-4.el test-load-4.wel test-load-4.graph \
					 test-load-4w.graph test-load-4.mtx test-load-4w.mtx

test/out/load-%.out: test/out $(GENERATE_KERNEL)
	./$(GENERATE_KERNEL) -f test/graphs/$* -n0 > $@

.SECONDARY: # want to keep all intermediate files (test outputs)
test-load-%: test/out/load-%.out
	@if grep -q "`cat test/reference/graph-$*.out`" $<; \
		then echo " $(PASS) Load $*"; \
		else echo " $(FAIL) Load $*"; \
	fi



# Kernel Output Verification -------------------------------------------#
#-----------------------------------------------------------------------#

# Trivally small graph, will add benchmark graphs
TEST_GRAPH ?= g10

test/out/verify-%-$(TEST_GRAPH).out: test/out %
	./$* -$(TEST_GRAPH) -vn1 > $@

.SECONDARY:
test-verify-%-$(TEST_GRAPH): test/out/verify-%-$(TEST_GRAPH).out
	@if grep -q "Verification:           PASS" $<; \
		then echo " $(PASS) Verify $*"; \
		else echo " $(FAIL) Verify $*"; \
	fi

test-verify: $(addsuffix -$(TEST_GRAPH), $(addprefix test-verify-, $(KERNELS)))
