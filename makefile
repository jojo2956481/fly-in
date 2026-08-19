PYTHON = python3

run:
	@uv run python -m src $(ARGS)

install:
	@uv add flake8 mypy pydantic

debug:
	@$(PYTHON) -m pdb src/main.py

clean:
	@rm -rf */*/__pycache__/ */*/*/__pycache__ */*__pycache__
	@rm -rf .mypy_cache/

lint:
	@flake8 src/
	@mypy src/ --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs --follow-imports=skip

lint-strict:
	@mypy src/ --strict --follow-imports=silent

.PHONY: 
	install
	run
	debug
	clean
	lint
	lint-strict
