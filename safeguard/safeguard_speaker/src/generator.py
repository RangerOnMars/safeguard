from gtts import gTTS
text = "已获取目标点位"
tts = gTTS(text=text, lang="zh-CN")
tts.save(text +".mp3")
