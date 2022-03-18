# Primitive types
bool bool_member true
int8 int8_member 1
uint8 uint8_member 2
byte byte_member 3
float32 float32_member 1e-2

# Array/sequence of primitive type
float32[3] float_array [1.0, 2.0, 3.0]
float32[<=3] float_seq_bounded [4.0, 5.0]
float32[] float_seq_unbounded [6.0]

# String types
string string_member "Χαίρετε 你好"
wstring wstring_member "αντίο σου 再见"
string<=3 bounded_string_member "äöü"
wstring<=3 bounded_wstring_member "äöü"

# Array/sequence of string type
string[4] string_array ["R", "O", "S", "2"]
string[<=4] string_seq_bounded ["R", "O", "S", "2"]
string[] string_seq_unbounded ["R", "O", "S", "2"]
string<=1[4] bounded_string_array ["R", "O", "S", "2"]
string<=1[<=4] bounded_string_seq_bounded ["R", "O", "S", "2"]
string<=1[] bounded_string_seq_unbounded ["R", "O", "S", "2"]

# Nested type
NestedType nested_member

# Array/sequence of nested type
NestedType[2] nested_array
NestedType[] nested_seq_unbounded
NestedType[<=3] nested_seq_bounded

