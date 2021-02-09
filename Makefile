version?=latest

build-base:
	docker build -f aerial_mapper/docker/Dockerfile.base --tag ethzasl/aerial_mapper:base-${version} .

push-base:
	docker push ethzasl/aerial_mapper:base-${version}

build:
	docker build -f aerial_mapper/docker/Dockerfile --tag ethzasl/aerial_mapper:${version} .

push:
	docker push ethzasl/aerial_mapper:${version}

publish: build push
publish-base: build-base push-base

run:
	docker run -it --rm ethzasl/aerial_mapper:${version} /bin/bash
