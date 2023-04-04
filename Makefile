.PHONY: default
default: test doc
	true

.PHONY: test
test:
	go test -v -coverprofile cover.out .
	go tool cover -html=cover.out -o cover.html

.PHONY: doc
doc:
	@go doc -all | sed 's/[{]/\n{\n/g' | sed 's/[}]/\n}\n/g' | sed 's/type/### type/g' | sed 's/func /### func /g' | sed 's/TYPES/## REFERENCE/g' | tee README.md

.PHONY: clean
clean:
	rm -f cover.out cover.html
