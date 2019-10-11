BUILD=go build
CLEAN=go clean
BUILDPATH=_build
GO111MODULE=on
EXAMPLES=$(wildcard */)

all: dep build

builddir:
	for example in $(EXAMPLES); do \
		mkdir -p $$example/$(BUILDPATH); \
	done

clean:
	for example in $(EXAMPLES); do \
		rm -rf $$example/$(BUILDPATH); \
	done

dep:
	go get ./...

build: builddir
	for example in $(EXAMPLES); do \
		go build -o $$example/$(BUILDPATH)/out $$example/*.go ; \
	done

travis: dep
	go build ./...

.PHONY: clean
