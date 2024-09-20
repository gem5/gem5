# GPU

These tests do random checks to the Ruby GPU protocol within gem5.
To run these tests by themselves, you can run the following command in the tests directory:

```bash
./main.py run gem5/gem5-resources --length=very-long
```

To obtain the input graphs needed to run the Pannotia tests, run the following
in tests/gpu-pannotia-resources/pannotia-datasets:

```bash
gsutil -m cp -r \
"gs://dist.gem5.org/dist/develop/datasets/pannotia/README.txt" \
"gs://dist.gem5.org/dist/develop/datasets/pannotia/bc" \
"gs://dist.gem5.org/dist/develop/datasets/pannotia/color" \
"gs://dist.gem5.org/dist/develop/datasets/pannotia/floydwarshall" \
"gs://dist.gem5.org/dist/develop/datasets/pannotia/mis" \
"gs://dist.gem5.org/dist/develop/datasets/pannotia/pagerank" \
"gs://dist.gem5.org/dist/develop/datasets/pannotia/sssp" \
.
```
