string = "ca_t::dog"
one, _ = string:match("([^,]+)::([^,]+)")
str = "dd7d07dd0d"
print(str)
str, s = str:match("([a-zA-Z]*)([0-9]*)")
print(str, s)
