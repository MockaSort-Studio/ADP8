from idl_parser import parser
parser_ = parser.IDLParser()

with open("core/communication/test/idl/Test.idl", "r") as f:
    idl_content = f.read()

global_module = parser_.load(idl_content)
print(global_module)
# my_module = global_module.module_by_name('my_module')
# my_struct = my_module.struct_by_name('MyPayload')

# for member in my_struct.members:
#     print(f"Field: {member.name}, Type: {member.type}")


if __name__ == "__main__":
    print("KTM")