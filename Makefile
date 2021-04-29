
.PHONY: release-linux

release-linux:
	cargo build --release
	strip target/release/sysroot-creator
	mkdir -p release
	tar -C ./target/release -czvf ./release/sysroot-creator-linux.tar.gz ./sysroot-creator
	ls -lh release