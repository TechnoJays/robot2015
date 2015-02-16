"""This module provides JSON helper functions."""

import json


def to_json(obj):
    """Convert an object to JSON.

    Args:
        obj: the object to convert.

    Returns:
        JSON-encoded object string.
    """
    def serialize(obj):
        """Recursive method to iterate through an object's properties.

        Args:
            obj: the object to iterate on.

        Returns:
            The JSON string representation of the object.
        """
        # Base types are fine
        if isinstance(obj, (bool, int, float, str)):
            return obj
        # Recursively iterate through dictionaries
        elif isinstance(obj, dict):
            obj = obj.copy()
            for key in obj:
                obj[key] = serialize(obj[key])
            return obj
        # Recursively iterate through lists
        elif isinstance(obj, list):
            return [serialize(item) for item in obj]
        # Recursively iterate through tuples
        elif isinstance(obj, tuple):
            return tuple(serialize([item for item in obj]))
        # For other objects, serialize the dictionary of its properties
        elif hasattr(obj, '__dict__'):
            return serialize(obj.__dict__)
        # All else just pass through as a string
        else:
            return repr(obj)

    # Serialize the object and convert to JSON
    return json.dumps(serialize(obj))

