PYTHON = python3

run:
	@uv run python -m src $(ARGS)

install:
	@uv add flake8 mypy pydantic pygame

debug:
	@$(PYTHON) -m pdb src/main.py

clean:
	@rm -rf */*/__pycache__/ */*/*/__pycache__ */*__pycache__
	@rm -rf .mypy_cache/

lint:
	@uv run flake8 src/
	@uv run mypy src/ --warn-return-any --warn-unused-ignor

lint-strict:
	@uv run mypy src/ --strict --follow-imports=silent

.PHONY: 
	install
	run
	debug
	clean
	lint
	lint-strict
