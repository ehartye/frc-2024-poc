const {spawn}=require("node:child_process");
process.stdin.setRawMode(true);process.stdin.setEncoding("utf8");let secret="";
process.stdout.write("Ready for private Site readback.\n");
process.stdin.on("data", chunk=>{secret+=chunk;if(!/[\r\n]/.test(secret))return;process.stdin.pause();process.stdin.setRawMode(false);const child=spawn(process.execPath,[__dirname+"/gallery-qa.cjs","https://frc-design-snapshot-gallery.ehartye.chatgpt.site"],{stdio:["pipe","inherit","inherit"]});child.stdin.end(secret.trim());secret="";child.on("exit", code=>process.exit(code));});
