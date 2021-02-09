version?=latest

build:
	docker build -f aerial_mapper/docker/Dockerfile --tag ethzasl/aerial_mapper:${version} .

push:
	docker push ethzasl/aerial_mapper:${version}

publish: build push
