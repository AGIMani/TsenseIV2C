"""Password-free LAN access with explicit Host/Origin validation."""
from urllib.parse import urlsplit


class AccessControl:
    def __init__(self, origins, encoded_password="", secure=False):
        # Old configuration/password arguments are deliberately ignored.
        self.origins = {origin.rstrip("/") for origin in origins}
        self.authorities = {urlsplit(origin).netloc for origin in self.origins}

    @property
    def required(self):
        return False

    def authenticated(self, headers):
        return True

    def token(self, headers):
        # Old login cookies must not change a tab's control identity.
        return ""

    def allowed(self, headers):
        return (headers.get("Host", "") in self.authorities
                and (not headers.get("Origin") or headers["Origin"].rstrip("/") in self.origins)
                and headers.get("Sec-Fetch-Site") != "cross-site")
