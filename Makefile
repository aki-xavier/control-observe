# control-observe — the estimator layer: the obstacle filter, the belief the EKF updates, the pose-motor observation.
#
#   make test          # this crate's suite, then the comment rules over the tree
#   make comments      # the comment rules alone, with the local approximation for the rest
#
# The comment rules are a dev-dependency of this crate: they read text, and nothing here links them. This
# crate's sources carry few comment blocks, so the rules have little to say here — the target exists so
# the few stay honest.
#
# Every Cargo command below is run as `mbx <subcommand>` (the build-cache wrapper). The rules live in
# ../comment-why, read text, and need no toolchain of their own: the gate inside `mbx test` is
# tests/comment_why.rs, and `make comments` is the same rules over the working tree, with that crate's
# local approximation for the comments the rules cannot decide.

COMMENT_WHY ?= ../comment-why

.PHONY: test comments

test:
	mbx test
	$(MAKE) comments

comments:
	mbx run --quiet --manifest-path $(COMMENT_WHY)/Cargo.toml --bin comment-why -- --review
