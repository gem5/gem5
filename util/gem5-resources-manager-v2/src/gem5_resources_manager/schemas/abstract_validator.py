from abc import ABC


class AbstractValidator(ABC):
    def __init__(self, schema) -> None:
        super().__init__()
        self.schema = schema

    def validate(self, data):
        raise NotImplementedError("validate() not implemented")

    def get_schema(self):
        return self.schema

    def get_fields(self, category):
        raise NotImplementedError("get_fields() not implemented")
